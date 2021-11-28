#pragma once

#include "Socket.h"
#include "RosBridgeMsg.h"
#include "Publisher.h"

#include <string>

namespace ros
{   

    class NodeHandle 
    {
        public:
            NodeHandle();
            NodeHandle(const NodeHandle&) = delete;

            template <typename T> Publisher advertise(std::string const& topic);

        private:
            void _send_ros_msg(RosBridgeMsg &);

            Socket *sock;
    };

    template <typename T> Publisher NodeHandle::advertise(std::string const& topic)
    {
        Publisher new_pub(*this);

        Advertise msg("advertise", topic, "std_msgs/String");

        _send_ros_msg(msg);

        return new_pub;
    }
}

