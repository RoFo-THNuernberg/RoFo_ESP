#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <string>
#include <vector>
#include <cstring>


enum SOCKET_STATUS 
{
    SOCKET_FAIL = -1,
    SOCKET_OK = 1
};


class Socket 
{          
    public:
        Socket(int port, std::string ip_addr);
        ~Socket();

        int socket_send(uint8_t const* tx_buffer, int send_bytes);
        int socket_receive(uint8_t* rx_buffer, int recv_bytes);
        int socket_receive_nonblock(uint8_t* rx_buffer, int recv_bytes);
        int socket_receive_string(std::string& new_string, int max_bytes);

        void connect_socket();    
        void disconnect_socket();

        bool sendFailed();

    private:
        Socket(const Socket&) = delete;

        int _connection_fd;

        bool _send_failed = false;

        int const _socket_port;
        std::string _ip_addr;

        struct sockaddr_in _server_socket;

        SemaphoreHandle_t _send_mutx;
};
