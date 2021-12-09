#pragma once

#include <string>
#include <functional>

#include "RosMsgs.h"

namespace ros
{
    class Subscriber
    {
        public:
            Subscriber(std::string topic, ros_msgs::RosMsg& msg_type, std::function<void(ros_msgs::RosMsg const&)> callback_function) : 
                topic{topic}, msg_type{msg_type}, callback_function{callback_function} {}
            ~Subscriber()
            {
                delete &msg_type;
            }

            std::string topic;
            ros_msgs::RosMsg& msg_type;
            std::function<void(ros_msgs::RosMsg const&)> callback_function;
    };
}