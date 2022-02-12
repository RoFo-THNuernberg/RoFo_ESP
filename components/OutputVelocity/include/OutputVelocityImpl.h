#pragma once

#include "OutputVelocity.h"
#include "MotorController.h"

#include "math.h"

/**
 * @brief Transforms velocity vector (m/s, RAD/s) to motor speeds (RAD/s) and sets the velocities of the MotorController object
 */
class OutputVelocityImpl : public OutputVelocity
{
    public:
        /**
         * @brief Initialize the OutputVelocityImpl instance
         * 
         * @note It is safe to call this function multiple times. It will only create one instance.
         * 
         * @return Reference to OutputVelocity instance
         */ 
        static OutputVelocity& init(MotorController& motor_controller);

        /**
         * @brief This function transform does the differential drive forward kinematic and passes the resulting setpoint motor velocities to the MotorController object
         * 
         * @param velocity velocity vector
         */  
        void setVelocity(ros_msgs_lw::Twist2D const& velocity) override;

    private:
        OutputVelocityImpl(MotorController& motor_controller);
        OutputVelocityImpl(OutputVelocityImpl const&) = delete;
        ~OutputVelocityImpl() {}

        MotorController& _motor_controller;
};