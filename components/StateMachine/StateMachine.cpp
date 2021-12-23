#include "StateMachine.h"
#include "States.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#define TAG "StateMachine"

StateMachine::StateMachine() : _current_state{new Idle} {}

void StateMachine::setState(State* new_state)
{
    ESP_LOGV(TAG, "%s -> %s", _current_state->getState().c_str(), new_state->getState().c_str());

    delete _current_state;
    _current_state = new_state;
}

void StateMachine::set_velocity(ros_msgs::RosMsg const& vel_vector)
{
    _current_state->set_velocity(*this, (ros_msgs::Twist2D&)vel_vector);
}

void StateMachine::set_goal_point(ros_msgs::RosMsg const& goal_point)
{
    _current_state->set_goal_point(*this, (ros_msgs::Point2D&)goal_point);
}

void StateMachine::stop()
{
    _current_state->stop(*this);
}