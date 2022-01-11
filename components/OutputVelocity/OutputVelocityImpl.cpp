#include "OutputVelocityImpl.h"

#define WHEEL_BASE_DISTANCE 0.13
#define WHEEL_RADIUS 0.07

OutputVelocityImpl::OutputVelocityImpl() : _motor_a{MotorController::getMotorControllerA()}, _motor_b{MotorController::getMotorControllerB()} {}

OutputVelocity& OutputVelocityImpl::init()
{
    if(_output_velocity == nullptr)
        _output_velocity = new OutputVelocityImpl();

    return *_output_velocity;
}

void OutputVelocityImpl::setVelocity(ros_msgs::Twist2D const& velocity)
{
    _current_velocity = velocity;

    double motor_a_rps = (velocity.v - WHEEL_BASE_DISTANCE / 2 * velocity.w) / (2 * M_PI * WHEEL_RADIUS);
    double motor_b_rps = -(velocity.v + WHEEL_BASE_DISTANCE / 2 * velocity.w) / (2 * M_PI * WHEEL_RADIUS);

    _motor_a.setVelocity(motor_a_rps);
    _motor_b.setVelocity(motor_b_rps);
}