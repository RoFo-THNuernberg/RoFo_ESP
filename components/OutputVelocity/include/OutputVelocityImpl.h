#pragma once

#include "OutputVelocity.h"
#include "MotorController.h"

#include "math.h"

class OutputVelocityImpl : public OutputVelocity
{
    public:
        static OutputVelocity& init();
        void setVelocity(ros_msgs::Twist2D const& velocity) override;

    private:
        OutputVelocityImpl();
        OutputVelocityImpl(OutputVelocityImpl const&);
        ~OutputVelocityImpl() {}

        MotorController& _motor_a;
        MotorController& _motor_b;
};