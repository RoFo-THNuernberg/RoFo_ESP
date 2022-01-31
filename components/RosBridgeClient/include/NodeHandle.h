#pragma once

#include "Socket.h"
#include "RosMsgs.h"
#include "Publisher.h"
#include "Subscriber.h"
#include "msg_id.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace ros
{  

    class NodeHandle 
    {
        public:
           static NodeHandle& init(std::string ros_namespace, Socket& sock);

            template <typename T> Publisher<T> advertise(std::string const& topic);
            template <typename T> void subscribe(std::string const& topic, std::function<void(std::shared_ptr<T> ros_msg)> callback_function);

        private:
        	NodeHandle(std::string ros_namespace, Socket& sock);
            NodeHandle(const NodeHandle&) = delete;
            ~NodeHandle();

            int _send_init();
            int _send_keep_alive();
            void _restart_protocol();
            int _interpret_receive();

            static void _communication_handler(void* arg);
            static void _check_keep_alive(void *arg);

            Subscriber* _getSubscriber(std::string const& topic);
            void _unsubscribe();

            static NodeHandle* _node_handle_obj;

            Socket& _sock;
            std::string _ros_namespace;
            std::vector<Subscriber*> _subscriber;

            static TaskHandle_t _communication_handler_thread;
            static TaskHandle_t _check_keep_alive_thread;

            static SemaphoreHandle_t _restart_mutx;

            bool _protocol_restarting = false;

            uint64_t _server_time_difference_us;
            uint64_t _keep_alive_time_us;
            uint64_t _last_send_keep_alive_us;
    };

    template <typename T> Publisher<T> NodeHandle::advertise(std::string const& topic)
    {
        Publisher<T> new_pub(topic, _sock);

        return new_pub;
    }

    template <typename T> void NodeHandle::subscribe(std::string const& topic, std::function<void(std::shared_ptr<T> ros_msg)>callback_function)
    {   
        Subscriber* new_sub = new SubscriberImpl<T>(topic, _sock, callback_function);

        _subscriber.push_back(new_sub);
    }

}

