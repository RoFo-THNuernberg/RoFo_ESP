#include "Socket.h"

 
#define SOCKET_PORT CONFIG_SOCKET_PORT
#define SOCKET_PROTOCOL CONFIG_SOCKET_PROTOCOL
#define SERVER_IP_ADDR CONFIG_SERVER_IP_ADDR
#define TX_QUEUE_SIZE CONFIG_TX_QUEUE_SIZE

#define TAG "Socket"

Socket* Socket::_sock_obj = nullptr;
QueueHandle_t Socket::_tx_queue{};

Socket::Socket() :  _port{SOCKET_PORT} , _protocol{SOCKET_PROTOCOL}, _server_ip{SERVER_IP_ADDR}
{
    _server_socket.sin_addr.s_addr = inet_addr(_server_ip.c_str());
    _server_socket.sin_family = AF_INET;
    _server_socket.sin_port = htons(_port);

    _tx_queue = xQueueCreate(TX_QUEUE_SIZE, sizeof(SocketPaket*));

    _connect_socket();

    xTaskCreate(_send_data_task, "write_data_task", 4096, this, 5, NULL);
}

Socket* Socket::init()
{
    if (_sock_obj == nullptr)
    {   
        ESP_LOGI(TAG, "Creating Socket Object...");
        _sock_obj = new Socket();
    }

    return _sock_obj;
}

esp_err_t Socket::send_data(SocketPaket const* new_paket)
{   
    esp_err_t status = ESP_OK;

    if(pdPASS != xQueueSend(_tx_queue, &new_paket, 0))
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

            int err = connect(_socket_fd, (struct sockaddr*)&_server_socket, sizeof(struct sockaddr_in));
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

        SocketPaket *paket;

        xQueueReceive(_tx_queue, &paket, portMAX_DELAY);

        if(paket != nullptr)
        {
            int err;

            do 
            {
                err = send(this_sock->_socket_fd, paket->buffer, paket->buffer_len, 0);

                if(err < 0)
                {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);

                    if(errno != ENOMEM)
                        this_sock->_restart_socket();
                }
            } 
            while(err < 0);

            delete paket;
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

}
