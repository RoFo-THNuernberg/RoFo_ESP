#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <string>
#include <vector>
#include <cstring>


enum SOCKET_STATUS {
    SOCKET_FAIL = -1,
    SOCKET_OK = 1
};


class Socket 
{          
    public:
        static Socket& init();

        int socket_send(uint8_t* tx_buffer, int send_bytes);
        int socket_receive(uint8_t* rx_buffer, int recv_bytes);
        int socket_receive_nonblock(uint8_t* rx_buffer, int recv_bytes);
        int socket_receive_string(std::string& new_string, int max_bytes);

        void restart_socket();

    private:
        Socket();
        Socket(const Socket&) = delete;
        ~Socket() {}

        void _connect_socket();    
        void _disconnect_socket();

        static Socket* _sock_obj;

        int _connection_fd;
        struct sockaddr_in _server_socket;
};
