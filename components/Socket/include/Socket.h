#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    private:
        Socket(const Socket&) = delete;

        int _port;
        std::string _ip_addr;

        int _connection_fd;
        struct sockaddr_in _server_socket;
};
