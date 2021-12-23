#include "OutputVelocitySim.h"

OutputVelocitySim::OutputVelocitySim(ros::NodeHandle& node_handle) : _publisher{node_handle.advertise<ros_msgs::Twist2D>("cmd_vel")} {}

OutputVelocity& OutputVelocitySim::init(ros::NodeHandle& node_handle)
{
    if(_output_velocity == nullptr)
        _output_velocity = new OutputVelocitySim(node_handle);

    return *_output_velocity;
}

void OutputVelocitySim::setVelocity(ros_msgs::Twist2D const& velocity)
{
    _publisher.publish(velocity);
}