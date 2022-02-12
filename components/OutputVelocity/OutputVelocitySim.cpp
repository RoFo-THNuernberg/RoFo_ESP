#include "OutputVelocitySim.h"

OutputVelocitySim::OutputVelocitySim(ros::Publisher<ros_msgs::Twist2D>& publisher) : _publisher{publisher} {}

OutputVelocity& OutputVelocitySim::init(ros::Publisher<ros_msgs::Twist2D>& publisher)
{
    if(_output_velocity_obj == nullptr)
        _output_velocity_obj = new OutputVelocitySim(publisher);

    return *_output_velocity_obj;
}

void OutputVelocitySim::setVelocity(ros_msgs_lw::Twist2D const& velocity)
{   
    _current_velocity = velocity;
    ros_msgs::Twist2D vel_msg(velocity);

    _publisher.publish(vel_msg);
}