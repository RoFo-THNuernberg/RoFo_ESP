#include "NodeHandle.h"

namespace ros
{   
    NodeHandle::NodeHandle() : sock{Socket::init()} {}

    void NodeHandle::_send_ros_msg(RosBridgeMsg& new_msg)
    {
        sock->send_data(new_msg.getJSON());
    }
    
}

