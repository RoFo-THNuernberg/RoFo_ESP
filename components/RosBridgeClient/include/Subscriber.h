#pragma once

#include <string>

#include "RosMsgs.h"

namespace ros
{
    class Subscriber
    {
        public:
            Subscriber(std::string topic, ros_msgs::RosMsg& msg_type, void (*callback_function)(ros_msgs::RosMsg&)) : 
                topic{topic}, msg_type{msg_type}, callback_function{callback_function} {}

            std::string topic;
            ros_msgs::RosMsg& msg_type;
            void (*callback_function)(ros_msgs::RosMsg&);
    };
}