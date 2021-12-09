#include "StateMachine.h"
#include "States.h"


StateMachine::StateMachine() : _current_state{new Idle} {}

void StateMachine::setState(State* new_state)
{
    delete _current_state;
    _current_state = new_state;
}

void StateMachine::set_velocity(ros_msgs::RosMsg const& vel_vector)
{
    _current_state->set_velocity(*this, (ros_msgs::Twist2D&)vel_vector);
}

void StateMachine::set_goal_point(ros_msgs::RosMsg const& goal_point)
{
    ESP_LOGI("test", "hello2");
    _current_state->set_goal_point(*this, (ros_msgs::Point2D&)goal_point);
}

void StateMachine::stop(ros_msgs::RosMsg const& stop_msg)
{
    _current_state->stop(*this, (ros_msgs::String&)stop_msg);
}