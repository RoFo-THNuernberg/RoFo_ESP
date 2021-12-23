#pragma once

#include "RosMsgs.h"

class OutputVelocity
{
    public:
        virtual ~OutputVelocity() {}

        static OutputVelocity& getOutput();

        virtual void setVelocity(ros_msgs::Twist2D const& velocity) = 0;

    protected:
        static OutputVelocity* _output_velocity;


};
