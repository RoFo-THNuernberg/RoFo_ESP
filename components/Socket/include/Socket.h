#pragma once

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"

#include <string>
#include <cstring>

namespace SOCKET
{

    class Socket {
        public:
            Socket();
            ~Socket();
            esp_err_t init();
            esp_err_t write_periodic(const std::string& msg_name);
            esp_err_t send_data(const std::string& msg_name, char* _data_buf);

        private:
            void _connect_socket();     //ToDo: implement resource protection for multi-task access
            void _disconnect_socket();
            void _restart_socket();

            static void _send_data_task(void *arg);

            char *_tx_buffer;
            int _tx_buffer_len = 0;

            static SemaphoreHandle_t _tx_buf_mutx;

            int _socket_fd;
            const int _port;
            const std::string _protocol;
            const std::string _server_ip;
            struct sockaddr_in _dest_addr;

    };
};