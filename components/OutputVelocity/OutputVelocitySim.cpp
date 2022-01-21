#include "OutputVelocitySim.h"

OutputVelocitySim::OutputVelocitySim(ros::NodeHandle& node_handle) : _publisher{node_handle.advertise<ros_msgs::Twist2D>("cmd_vel")} {}

OutputVelocity& OutputVelocitySim::init(ros::NodeHandle& node_handle)
{
    if(_output_velocity_obj == nullptr)
        _output_velocity_obj = new OutputVelocitySim(node_handle);

    return *_output_velocity_obj;
}

void OutputVelocitySim::setVelocity(ros_msgs_lw::Twist2D const& velocity)
{   
    _current_velocity = velocity;
    ros_msgs::Twist2D vel_msg(velocity);

    _publisher.publish(vel_msg);
}