#include "NodeHandle.h"

namespace ros
{   
    NodeHandle::NodeHandle(std::string node_name) : _sock{Socket::init()}, _node_name{node_name} 
    {
        uint8_t *pkt_buffer = new uint8_t[_node_name.size() + 2];
        int pkt_len = 0;

        pkt_buffer[0] = INIT_ID;
        pkt_len++;

        memcpy(pkt_buffer + pkt_len, _node_name.c_str(), _node_name.size());
        pkt_len += _node_name.size();

        pkt_buffer[pkt_len] = '\0';
        pkt_len++;
        
        SocketPaket *new_pkt = new SocketPaket(pkt_buffer, pkt_len);
        _sock->send_data(new_pkt);
    }

}

