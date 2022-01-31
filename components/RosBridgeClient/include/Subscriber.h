#pragma once

#include <string>
#include <functional>
#include <memory>

#include "RosMsgs.h"
#include "Socket.h"

namespace ros
{  

    class Subscriber
    {
        public:
            virtual ~Subscriber() {}

            virtual bool recvMessage() = 0;
            virtual bool compareTopic(std::string const& topic) = 0;

    };

    template <typename T> class SubscriberImpl : public Subscriber
    {
        public:
            SubscriberImpl(std::string const& topic, Socket& sock, std::function<void(std::shared_ptr<T> ros_msg)> callback_function) : 
                _topic{topic}, _sock{sock}, _callback_function{callback_function} {}
            ~SubscriberImpl() {}

            bool recvMessage() override
            {
                std::shared_ptr<T> ros_msg = std::make_shared<T>();

                int32_t msg_len = ros_msg->getSize();

                if(msg_len == 0)
                {
                    if(_sock.socket_receive((uint8_t*)&msg_len, sizeof(msg_len)) == SOCKET_FAIL)
                        return false;
                }

                uint8_t* rx_buffer = new uint8_t[msg_len];
                if(_sock.socket_receive(rx_buffer, msg_len) == SOCKET_FAIL)
                    return false;

                ros_msg->deserialize(rx_buffer);

                _callback_function(ros_msg);

                delete[] rx_buffer;

                return true;
            }

            bool compareTopic(std::string const& topic)
            {
                return _topic == topic;
            }

            
        private:
            std::string const _topic;
            Socket& _sock;
            std::function<void(std::shared_ptr<T> ros_msg)> _callback_function;

    };
}