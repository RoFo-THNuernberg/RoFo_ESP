#include "States.h"
#include "StateMachine.h"

#include "esp_log.h"

void Idle::set_velocity(StateMachine& state_machine, ros_msgs::Twist2D const& vel_vector)
{
    if((int)(vel_vector.x * vel_vector.x + vel_vector.w * vel_vector.w) != 0)
        state_machine.setState(new DriveWithVelocity);
}

void Idle::set_goal_point(StateMachine& state_machine, ros_msgs::Point2D const& goal_point)
{
    //Set goal point
    state_machine.setState(new DriveToPoint);
    ESP_LOGI("idle", "set_goal_point");
}

void DriveWithVelocity::set_velocity(StateMachine& state_machine, ros_msgs::Twist2D const& vel_vector)
{
    if((int)(vel_vector.x * vel_vector.x + vel_vector.w * vel_vector.w) == 0)
        state_machine.setState(new Idle);
    else
    {
        //change speed
    }
}

void DriveWithVelocity::stop(StateMachine& state_machine, ros_msgs::String const& stop_msg)
{
    state_machine.setState(new Idle);
}

void DriveToPoint::set_goal_point(StateMachine& state_machine, ros_msgs::Point2D const& goal_point)
{
    //set goal point
    ESP_LOGI("state", "set_goal_point");
}

void DriveToPoint::stop(StateMachine& state_machine, ros_msgs::String const& stop_msg)
{
    state_machine.setState(new Idle);
}