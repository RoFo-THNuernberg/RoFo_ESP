#include "OutputVelocity.h"

OutputVelocity* OutputVelocity::_output_velocity = nullptr;

OutputVelocity& OutputVelocity::getOutput()
{
    return *_output_velocity;
}

ros_msgs::Twist2D OutputVelocity::getVelocity()
{
    return _current_velocity;
}