#include "RosMsgs.h"

namespace ros_msgs
{
    Pose2D operator+(ros_msgs::Pose2D pose, std::array<float, 3> vector)
    {
        pose.x += vector[0];
        pose.y += vector[1];
        pose.theta += vector[2];

        //Keep theta between -pi and pi
        pose.theta = atan2(sin(pose.theta), cos(pose.theta));

        return pose;
    }
    
}