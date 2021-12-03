#pragma once

#include "Socket.h"
#include <vector>
#include <string>

namespace ros
{   
    #define PUBLISH_ID 3

    class Publisher
    {
        public:
            Publisher(const std::string topic) : _topic{topic}, _sock{Socket::init()}{}
            template <typename T> void publish(const T& );

        private:
            const std::string _topic;
            Socket *_sock;
    };


    template <typename T> void Publisher::publish(const T& msg)
    {   
        uint8_t *pkt_buffer = new uint8_t[_topic.size() + msg.getSize() + 2];
        int pkt_len = 0;
        
        pkt_buffer[0] = PUBLISH_ID;
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, _topic.c_str(), _topic.size());
        pkt_len += _topic.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, msg.getBuffer(), msg.getSize());
        pkt_len += msg.getSize();

        SocketPaket *new_pkt = new SocketPaket(pkt_buffer, pkt_len);
        

        _sock->send_data(new_pkt);
    }
}