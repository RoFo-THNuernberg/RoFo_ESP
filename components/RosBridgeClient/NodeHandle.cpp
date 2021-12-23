#include "NodeHandle.h"

namespace ros
{   
    #define TAG "NodeHandle"

    NodeHandle* NodeHandle::_node_handle_obj = nullptr;

    SemaphoreHandle_t NodeHandle::_restart_mutx{};
    TaskHandle_t NodeHandle::_check_keep_alive_thread{};
    TaskHandle_t NodeHandle::_communication_handler_thread{};

    NodeHandle::NodeHandle(std::string ros_namespace) : _sock{Socket::init()}, _ros_namespace{ros_namespace}
    {   
        _keep_alive_time_us = esp_timer_get_time();
        _last_send_keep_alive_us = esp_timer_get_time();

        _restart_mutx = xSemaphoreCreateMutex();
        
        _send_init();

        xTaskCreate(_communication_handler, "_communication_handler", 4048, this, 5, &_communication_handler_thread);
        xTaskCreate(_check_keep_alive, "_check_keep_alive", 2048, this, 5, &_check_keep_alive_thread);
    }

    NodeHandle::~NodeHandle()
    {   
        vTaskDelete(_communication_handler_thread);
        vTaskDelete(_check_keep_alive_thread);

        _unsubscribe();
    }

    NodeHandle& NodeHandle::init(std::string ros_namespace)
    {
        if(_node_handle_obj == nullptr)
            _node_handle_obj = new NodeHandle(ros_namespace);

        return *_node_handle_obj;
    }

    void NodeHandle::_restart_protocol()
    {   
        if(xSemaphoreTake(_restart_mutx, 0) == pdPASS)
        {   
            ESP_LOGI(TAG, "Restarting Protocol!");
            _protocol_restarting = true;

            _sock.restart_socket();
            _send_init();
            _keep_alive_time_us = esp_timer_get_time();

            _protocol_restarting = false;

            xSemaphoreGive(_restart_mutx);
        }
    }

    int NodeHandle::_send_init()
    {
        uint8_t pkt_buffer[_ros_namespace.size() + 2];
        int pkt_len = 0;

        pkt_buffer[0] = INIT_ID;
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, _ros_namespace.c_str(), _ros_namespace.size());
        pkt_len += _ros_namespace.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        ESP_LOGI(TAG, "Sending initialize message!");

        return _sock.socket_send(pkt_buffer, pkt_len);
    }

    int NodeHandle::_send_keep_alive()
    {   
        uint64_t time_now_us = esp_timer_get_time();

        if(_protocol_restarting == false && (time_now_us - _last_send_keep_alive_us) / 1000 > KEEP_ALIVE_SEND_PERIOD_MS)
        {
            uint8_t pkt_buffer[1 + sizeof(uint64_t)];
            int pkt_len = 0;

            pkt_buffer[0] = KEEP_ALIVE_ID;
            pkt_len++;

            *(uint64_t*)(pkt_buffer + pkt_len) = (uint64_t)time_now_us;
            pkt_len += sizeof(uint64_t);

            _last_send_keep_alive_us = time_now_us;

            return _sock.socket_send(pkt_buffer, pkt_len);
        }

        return -1;
    }

    void NodeHandle::_communication_handler(void* arg)
    {   
        NodeHandle* node_handle = (NodeHandle*)arg;

        while(1)
        {   
            if(node_handle->_send_keep_alive() != -1)
                if(node_handle->_interpret_receive() == -1)
                {
                    ESP_LOGE(TAG,"Interpret Receive failed!");
                    node_handle->_restart_protocol();
                }
                
            
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    

        vTaskDelete(NULL);
    }

    void NodeHandle::_check_keep_alive(void* arg)
    {
        NodeHandle* node_handle = (NodeHandle*)arg;

        while(1)
        {
            if((esp_timer_get_time() - node_handle->_keep_alive_time_us) / 1000 > MAX_KEEP_ALIVE_TIMOUT_MS)
            {
                ESP_LOGE(TAG,"Check Keep Alive Timeout!");
                node_handle->_restart_protocol();
            }

            vTaskDelay(pdMS_TO_TICKS(KEEP_ALIVE_CHECK_PERIOD_MS));
        }

        vTaskDelete(NULL);
    }

   int NodeHandle::_interpret_receive()
    {   
        int status_error = 0;

        while(1)
        {

            uint8_t msg_id;

            status_error = _sock.socket_receive_nonblock(&msg_id, 1);

            if(status_error == SOCKET_FAIL)
            {
                ESP_LOGE(TAG, "Error while receiving MSG ID");
                return status_error;
            } 
            else if (status_error == 0)
            {
                return status_error;
            }

            switch(msg_id)
            {   
                case KEEP_ALIVE_ID:
                {   
                    uint64_t server_time;
                    status_error = _sock.socket_receive((uint8_t*)&server_time, sizeof(server_time));

                    if(status_error == SOCKET_FAIL)
                        break;

                    _keep_alive_time_us = esp_timer_get_time();

                    _server_time_difference_us = server_time - _keep_alive_time_us;

                    ESP_LOGI(TAG, "Keep Alive! Server Time: %lld", server_time);

                    break;
                }
                case PUBLISH_ID:
                {   
                    std::string topic;
                    status_error = _sock.socket_receive_string(topic, 32);

                    if(status_error == SOCKET_FAIL)
                        break;

                    //ESP_LOGI(TAG, "Received topic: %s", topic.c_str());
                    
                    Subscriber* sub = _getSubscriber(topic);

                    if(sub != nullptr)
                    {   
                        int msg_len = sub->getMsg().getSize();

                        if(msg_len == -1)
                        {
                            status_error = _sock.socket_receive((uint8_t*)&msg_len, sizeof(msg_len));

                            if(status_error == SOCKET_FAIL)
                                break;
                        }

                        uint8_t* rx_buffer = new uint8_t[msg_len];
                        status_error = _sock.socket_receive(rx_buffer, msg_len);

                        if(status_error == SOCKET_FAIL)
                            break;

                        sub->getMsg().deserialize(rx_buffer);

                        delete[] rx_buffer;

                        sub->callback_function(sub->getMsg());
                    }
                    else
                        status_error = SOCKET_FAIL;

                    break;
                }
                default:
                    ESP_LOGE(TAG, "ID not found: %d", msg_id);
                    status_error = SOCKET_FAIL;
                    break;
            }

            if(status_error == SOCKET_FAIL)
                break;

        }

        return status_error;
    }

    Subscriber* NodeHandle::_getSubscriber(std::string const& topic)
    {
        for(auto i : _subscriber)
        {
            if(i->topic == topic)
            {
                return i;
            }
        }

        return nullptr;
    }

    void NodeHandle::_unsubscribe()
    {
        for(auto i : _subscriber)
            delete i;
    }
}