#include "States.h"
#include "StateMachine.h"

#define MIN_VEL_THRESHOLD 0.01

void Idle::set_velocity(StateMachine& state_machine, ros_msgs::Twist2D const& vel_vector)
{
    if((vel_vector.v * vel_vector.v + vel_vector.w * vel_vector.w) > MIN_VEL_THRESHOLD)
    {
        state_machine.setState(new DriveWithVelocity);
        OutputVelocity::getOutput().setVelocity(vel_vector);
    }
}

void Idle::set_goal_point(StateMachine& state_machine, ros_msgs::Point2D const& goal_point)
{
    ControllerMaster::start_controller(new p2pController(0.3, 0.4, goal_point), std::bind(&StateMachine::stop, &state_machine));

    state_machine.setState(new DriveToPoint);
}

void DriveWithVelocity::set_velocity(StateMachine& state_machine, ros_msgs::Twist2D const& vel_vector)
{
    if((vel_vector.v * vel_vector.v + vel_vector.w * vel_vector.w) < MIN_VEL_THRESHOLD)
        state_machine.setState(new Idle);
    else
        OutputVelocity::getOutput().setVelocity(vel_vector);
}

void DriveWithVelocity::stop(StateMachine& state_machine)
{
    state_machine.setState(new Idle);

    ros_msgs::Twist2D vel_vector(0, 0);
    OutputVelocity::getOutput().setVelocity(vel_vector);
}

void DriveToPoint::set_goal_point(StateMachine& state_machine, ros_msgs::Point2D const& goal_point)
{
    ControllerMaster::start_controller(new p2pController(0.1, 0.1, goal_point), std::bind(&StateMachine::stop, &state_machine));

    state_machine.setState(new DriveToPoint);
}

void DriveToPoint::stop(StateMachine& state_machine)
{   
    ControllerMaster::stop_controller();
    state_machine.setState(new Idle);
}