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


class Socket 
{          
    public:
        static Socket* init();

        int socket_send(uint8_t* tx_buffer, int send_bytes);
        int socket_receive(uint8_t* rx_buffer, int recv_bytes);

        void restart_socket();

    private:
        Socket();
        Socket(const Socket&) = delete;
        ~Socket();

        void _connect_socket();     //ToDo: implement resource protection for multi-task access
        void _disconnect_socket();

        static Socket* _sock_obj;

        int _connection_fd;
        const int _port;
        const std::string _server_ip;
        struct sockaddr_in _server_socket;
};
