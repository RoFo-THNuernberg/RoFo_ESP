#pragma once

#include <stdint.h>

#include "RosMsgs.h"

class PositionController
{
    public:
        virtual ~PositionController() {}
        virtual ros_msgs::Twist2D update(ros_msgs::Pose2D const& actual_pose) = 0;
        virtual bool destination_reached() = 0;
    
    private:

};