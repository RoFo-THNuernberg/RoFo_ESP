#pragma once

#include "Socket.h"
#include "ros_msgs.h"
#include "Publisher.h"

#include <string>
#include <map>

namespace ros
{  
    #define ADVERTISE_ID 2
    #define INIT_ID CONFIG_INIT_ID

    class NodeHandle 
    {
        public:
            NodeHandle(std::string node_name);
            NodeHandle(const NodeHandle&) = delete;

            template <typename T> Publisher advertise(std::string const& topic);

        private:
            Socket *_sock;
            std::string _node_name;
    };

    template <typename T> Publisher NodeHandle::advertise(std::string const& topic)
    {
        Publisher new_pub(topic);

        uint8_t *pkt_buffer = new uint8_t[T::msg_type.size() + topic.size() + 3];
        int pkt_len = 0;

        pkt_buffer[0] = ADVERTISE_ID;
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, topic.c_str(), topic.size());
        pkt_len += topic.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, T::msg_type.c_str(), T::msg_type.size());
        pkt_len += T::msg_type.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;
        
        SocketPaket *new_pkt = new SocketPaket(pkt_buffer, pkt_len);
        _sock->send_data(new_pkt);

        return new_pub;
    }
}

