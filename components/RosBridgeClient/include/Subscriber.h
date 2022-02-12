#pragma once

#include <string>
#include <functional>
#include <memory>

#include "msg_id.h"
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
            virtual void subscribe();

    };

    template <typename T> class SubscriberImpl : public Subscriber
    {
        public:
            SubscriberImpl(std::string const& topic, Socket& sock, std::function<void(std::shared_ptr<T> ros_msg)> callback_function) : 
                _topic{topic}, _sock{sock}, _callback_function{callback_function} {}
            ~SubscriberImpl() {}

            bool recvMessage() override;
            bool compareTopic(std::string const& topic) override;

            void subscribe() override;

            
        private:
            std::string const _topic;
            Socket& _sock;
            std::function<void(std::shared_ptr<T> ros_msg)> _callback_function;

    };


    template <typename T> bool SubscriberImpl<T>::recvMessage()
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

    template <typename T> bool SubscriberImpl<T>::compareTopic(std::string const& topic)
    {
        return _topic == topic;
    }

    template <typename T> void SubscriberImpl<T>::subscribe()
    {
        uint8_t* pkt_buffer = new uint8_t[1 + _topic.size() + 1 + T::getMsgType().size() + 1];

        int pkt_len = 0;

        pkt_buffer[0] = SUBSCRIBE_ID;
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, _topic.c_str(), _topic.size());
        pkt_len += _topic.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, T::getMsgType().c_str(), T::getMsgType().size());
        pkt_len += T::getMsgType().size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        _sock.socket_send(pkt_buffer, pkt_len);

        delete[] pkt_buffer;
    }
}