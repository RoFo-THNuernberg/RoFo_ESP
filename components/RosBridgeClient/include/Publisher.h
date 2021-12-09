#pragma once

#include "Socket.h"
#include "msg_id.h"
#include "RosMsgs.h"

#include <vector>
#include <string>

namespace ros
{   

    template <typename T> class Publisher
    {
        public:
            Publisher(const std::string topic) : _topic{topic}, _sock{Socket::init()}{}
            void publish(const T& msg);

        private:
            const std::string _topic;
            Socket *_sock;
    };


    template <typename T> void Publisher<T>::publish(const T& msg)
    {   
        uint8_t *pkt_buffer = new uint8_t[_topic.size() + msg.getSize() +  2];
        int pkt_len = 0;
        
        pkt_buffer[0] = msg.getMsgType();
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, _topic.c_str(), _topic.size());
        pkt_len += _topic.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        msg.serialize(pkt_buffer + pkt_len);
        pkt_len += msg.getSize();

        _sock->socket_send(pkt_buffer, pkt_len);
    }
}