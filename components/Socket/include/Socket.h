#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <string>
#include <vector>
#include <cstring>


struct SocketPaket 
{
    SocketPaket(uint8_t const *buf, size_t const buf_len) : buffer_len{buf_len}, buffer{buf} {};
    ~SocketPaket()
    {
        delete[] buffer;
    };

    size_t const buffer_len;
    uint8_t const *buffer;
};


class Socket 
{          
    public:
        static Socket* init();

        //write data into tx buffer queue; deletion of buffer is managed by Socket class
        esp_err_t send_data(SocketPaket const* new_paket);

    private:
        Socket();
        Socket(const Socket&) = delete;
        ~Socket();

        void _connect_socket();     //ToDo: implement resource protection for multi-task access
        void _disconnect_socket();
        void _restart_socket();

        static void _send_data_task(void *arg);

        static Socket* _sock_obj;

        static QueueHandle_t _tx_queue;

        int _socket_fd;
        const int _port;
        const std::string _protocol;
        const std::string _server_ip;
        struct sockaddr_in _server_socket;

};
