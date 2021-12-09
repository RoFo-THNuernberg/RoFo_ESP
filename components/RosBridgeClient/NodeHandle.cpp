#include "NodeHandle.h"

namespace ros
{   
    #define TAG "NodeHandle"

    SemaphoreHandle_t NodeHandle::_restart_mutx{};
    TaskHandle_t NodeHandle::_check_keep_alive_thread{};
    TaskHandle_t NodeHandle::_communication_handler_thread{};

    NodeHandle::NodeHandle(std::string ros_namespace) : _ros_namespace{ros_namespace} 
    {   
        _keep_alive_time_us = esp_timer_get_time();
        _last_send_keep_alive_us = esp_timer_get_time();

        _sock = Socket::init();

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

    void NodeHandle::_restart_protocol()
    {   
        if(xSemaphoreTake(_restart_mutx, 0) == pdPASS)
        {   
            ESP_LOGI(TAG, "Restarting Protocol!");
            _protocol_restarting = true;

            _sock->restart_socket();
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

        return _sock->socket_send(pkt_buffer, pkt_len);
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

            return _sock->socket_send(pkt_buffer, pkt_len);
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
                    node_handle->_restart_protocol();
                
            
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
                node_handle->_restart_protocol();

            vTaskDelay(pdMS_TO_TICKS(KEEP_ALIVE_CHECK_PERIOD_MS));
        }

        vTaskDelete(NULL);
    }

    int NodeHandle::_interpret_receive()
    {   
        int socket_len_err = 0;

        uint8_t* rx_buffer[64];
        int rx_buffer_length = 0;
        int rx_buffer_row = 0;

        do {
            rx_buffer[rx_buffer_row] = new uint8_t[RX_BUF_LEN];

            socket_len_err = _sock->socket_receive(rx_buffer[rx_buffer_row], RX_BUF_LEN);
            rx_buffer_length += socket_len_err;

            rx_buffer_row++;

        } while (socket_len_err == RX_BUF_LEN && rx_buffer_row < 64);

        SmartBufferPtr rx_buffer_ptr(rx_buffer, RX_BUF_LEN, rx_buffer_row);
            

        while(rx_buffer_length > rx_buffer_ptr - SmartBufferPtr(rx_buffer, RX_BUF_LEN, rx_buffer_row) && socket_len_err != -1)
        {   
            switch(rx_buffer_ptr[0])
            {
                case KEEP_ALIVE_ID:
                {
                    rx_buffer_ptr += 1;

                    uint64_t server_time = 0;
                    server_time << rx_buffer_ptr;

                    rx_buffer_ptr += sizeof(uint64_t);

                    _keep_alive_time_us = esp_timer_get_time();

                    _server_time_difference_us = server_time - _keep_alive_time_us;

                    break;
                }
                case PUBLISH_ID:
                {
                    rx_buffer_ptr += 1;

                    std::string topic;
                    topic << rx_buffer_ptr;

                    rx_buffer_ptr += topic.size() + 1;

                    Subscriber* sub = _getSubscriber(topic);

                    if(sub != nullptr)
                    {
                        sub->msg_type.deserialize(rx_buffer_ptr);
                        rx_buffer_ptr += sub->msg_type.getSize();
                        ESP_LOGI("test", "hello1");
                        sub->callback_function(sub->msg_type);
                    } 
                    else
                        socket_len_err = -1;

                    break;
                }
                default:
                    socket_len_err = -1;
                    break;
            }
        }

        for(int i = 0; i < rx_buffer_row; i++)
            delete[] rx_buffer[i];

        return socket_len_err;
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