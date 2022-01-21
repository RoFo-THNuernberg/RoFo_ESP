#pragma once

#include "OutputVelocity.h"
#include "MotorController.h"

#include "math.h"

class OutputVelocityImpl : public OutputVelocity
{
    public:
        static OutputVelocity& init(MotorController& motor_controller);
        void setVelocity(ros_msgs_lw::Twist2D const& velocity) override;

    private:
        OutputVelocityImpl(MotorController& motor_controller);
        OutputVelocityImpl(OutputVelocityImpl const&) = delete;
        ~OutputVelocityImpl() {}

        MotorController& _motor_controller;
};