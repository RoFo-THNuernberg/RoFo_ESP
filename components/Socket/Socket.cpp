#include "Socket.h"

namespace SOCKET 
{   
    #define SOCKET_PORT CONFIG_SOCKET_PORT
    #define SOCKET_PROTOCOL CONFIG_SOCKET_PROTOCOL
    #define SERVER_IP_ADDR CONFIG_SERVER_IP_ADDR
    #define TX_BUFFER_SIZE CONFIG_TX_BUFFER_SIZE 

    #define TAG "Socket"

    SemaphoreHandle_t Socket::_tx_buf_mutx{};

    Socket::Socket() : _tx_buffer{new char[TX_BUFFER_SIZE]}, _port{SOCKET_PORT} , _protocol{SOCKET_PROTOCOL}, _server_ip{SERVER_IP_ADDR}{}

    Socket::~Socket() 
    {
        delete[] _tx_buffer;
        _tx_buffer = nullptr;
    }

    esp_err_t Socket::init()
    {
        esp_err_t status = ESP_OK;

        _dest_addr.sin_addr.s_addr = inet_addr(_server_ip.c_str());
        _dest_addr.sin_family = AF_INET;
        _dest_addr.sin_port = htons(_port);

        _tx_buf_mutx = xSemaphoreCreateMutex();

        _connect_socket();

        xTaskCreate(_send_data_task, "write_data_task", 4096, this, 5, NULL);

        return status;
    }

    esp_err_t Socket::send_data(std::string const& msg_name, char *data_buf)
    {   
        esp_err_t status = ESP_OK;

        int data_buf_len = strlen(data_buf);
        int new_data_len = msg_name.size() + data_buf_len + 3;

        if (_tx_buffer_len + new_data_len <= TX_BUFFER_SIZE)
        {   

            if(pdTRUE == xSemaphoreTake(_tx_buf_mutx, 0))
            {   
                _tx_buffer[_tx_buffer_len] = '[';
                _tx_buffer_len++;

                strcpy(_tx_buffer + _tx_buffer_len, msg_name.c_str());
                _tx_buffer_len += msg_name.size();

                _tx_buffer[_tx_buffer_len] = ',';
                _tx_buffer_len++;

                strcpy(_tx_buffer + _tx_buffer_len, data_buf);
                _tx_buffer_len += data_buf_len;

                _tx_buffer[_tx_buffer_len] = ']';
                _tx_buffer_len++;

                xSemaphoreGive(_tx_buf_mutx);
            }
            else
                status = ESP_FAIL;
        }
        else 
            status = ESP_FAIL;

        return status;
    }
    
    void Socket::_connect_socket()
    {   
        int i = 0;

        while(1) 
        {
            if(_protocol == "udp")
                _socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
            if(_protocol == "tcp")
                _socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

            if(_socket_fd >= 0)
            {
                ESP_LOGI(TAG, "Socket created, connecting to %s:%d", _server_ip.c_str(), _port);

                int err = connect(_socket_fd, (struct sockaddr*)&_dest_addr, sizeof(struct sockaddr_in));
                if (!err) 
                {
                    ESP_LOGI(TAG, "Successfully connected to %d", _port);
                    break;
                }

                ESP_LOGE(TAG, "Socket (%d) unable to connect: errno %d", _port, errno);
                _disconnect_socket();
            } 
            else 
                ESP_LOGE(TAG, "Unable to create socket (%d): errno %d", _port, errno);
            
            vTaskDelay( i*500 / portTICK_PERIOD_MS);
            i = (i < 8) ? i+1 : 10;   
        }   
    }

    void Socket::_disconnect_socket() 
    {
        if (_socket_fd != -1) {
            ESP_LOGE(TAG, "Shutting down socket (port: %d) and restarting...", _port);
            close(_socket_fd);
        }
    }

    void Socket::_restart_socket()
    {
        _disconnect_socket();

        vTaskDelay(5000 / portTICK_PERIOD_MS);

        _connect_socket();
    }

    void Socket::_send_data_task(void *arg)
    {
        Socket *this_sock = (Socket*)arg;
        while(1)
        {
            if(this_sock->_tx_buffer_len > 0)
            {
                xSemaphoreTake(_tx_buf_mutx, portMAX_DELAY);
                
                int err_len = send(this_sock->_socket_fd, this_sock->_tx_buffer, this_sock->_tx_buffer_len, 0);

                if (err_len >= 0) 
                {
                    this_sock->_tx_buffer_len = 0;
                    xSemaphoreGive(this_sock->_tx_buf_mutx);    
                } 
                else 
                {   
                    xSemaphoreGive(this_sock->_tx_buf_mutx);

                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);

                    if(errno != ENOMEM)
                        this_sock->_restart_socket();
                } 
            }

            vTaskDelay(50 / portTICK_PERIOD_MS);
        }

    }
}