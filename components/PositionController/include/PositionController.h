#pragma once

#include "RosMsgsLw.h"

/**
 * @brief Interface for PositionController classes 
 */
class PositionController
{
    public:
        virtual ~PositionController() {}
        virtual ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) = 0;
        virtual bool destination_reached() = 0;
    
    private:

};