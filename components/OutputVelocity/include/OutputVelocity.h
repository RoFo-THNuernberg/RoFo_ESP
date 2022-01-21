#pragma once

#include "RosMsgsLw.h"

class OutputVelocity
{
    public:
        virtual ~OutputVelocity() {}
        
        virtual void setVelocity(ros_msgs_lw::Twist2D const& velocity) = 0;
        ros_msgs_lw::Twist2D getVelocity() const;

    protected:
        static OutputVelocity* _output_velocity_obj;
        ros_msgs_lw::Twist2D _current_velocity;

};
