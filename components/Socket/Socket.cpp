#include "Socket.h"

 
#define SOCKET_PORT CONFIG_SOCKET_PORT
#define SERVER_IP_ADDR CONFIG_SERVER_IP_ADDR
#define TX_QUEUE_SIZE CONFIG_TX_QUEUE_SIZE

#define TAG "Socket"

Socket* Socket::_sock_obj = nullptr;

Socket::Socket() : _port{SOCKET_PORT},  _server_ip{SERVER_IP_ADDR}
{
    _server_socket.sin_addr.s_addr = inet_addr(_server_ip.c_str());
    _server_socket.sin_family = AF_INET;
    _server_socket.sin_port = htons(_port);

    _connect_socket();
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

void Socket::_connect_socket()
{   
    int i = 0;

    while(1) 
    {
        _connection_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

        if(_connection_fd != -1)
        {
            ESP_LOGI(TAG, "Socket created, connecting to %s:%d", _server_ip.c_str(), _port);

            int err = connect(_connection_fd, (struct sockaddr*)&_server_socket, sizeof(struct sockaddr_in));
            if (!err) 
            {
                ESP_LOGI(TAG, "Successfully connected to %d", _port);

                int flags = fcntl(_connection_fd, F_GETFL);
                fcntl(_connection_fd, F_SETFL, flags | O_NONBLOCK);

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
    if (_connection_fd != -1) {
        ESP_LOGE(TAG, "Shutting down socket (port: %d) and restarting...", _port);
        close(_connection_fd);
    }
}

void Socket::restart_socket()
{

    _disconnect_socket();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    _connect_socket();
}

int Socket::socket_receive(uint8_t* rx_buffer, int recv_bytes)
{
    int len = recv(_connection_fd, rx_buffer, recv_bytes, 0);

    if(len == -1 && errno == EWOULDBLOCK)
        len = 0;

    return len;
}

int Socket::socket_send(uint8_t* tx_buffer, int send_bytes)
{   
    int err = send(_connection_fd, tx_buffer, send_bytes, 0);

    if(err == -1 && errno == EWOULDBLOCK)
        err = 0;

    return err;
}