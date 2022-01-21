#include "States.h"
#include "StateMachine.h"

#define MIN_VEL_THRESHOLD 0.01

std::string const Idle::_state = "Idle";
std::string const DriveWithVelocity::_state = "DriveWithVelocity";
std::string const DriveToPoint::_state = "DriveToPoint";


void Idle::set_velocity(StateMachine& state_machine, ros_msgs_lw::Twist2D const& vel_vector)
{
    state_machine.output_velocity.setVelocity(vel_vector);

    if ((vel_vector.v * vel_vector.v + vel_vector.w * vel_vector.w) > MIN_VEL_THRESHOLD)
    {
        state_machine.setState(new DriveWithVelocity);
    }
}

void Idle::set_goal_point(StateMachine& state_machine, ros_msgs_lw::Point2D const& goal_point)
{
    state_machine.controller_master.start_controller(new p2pController(0.2, 0.8, goal_point), std::bind(&StateMachine::stop,& state_machine));

    state_machine.setState(new DriveToPoint);
}

void DriveWithVelocity::set_velocity(StateMachine& state_machine, ros_msgs_lw::Twist2D const& vel_vector)
{
    if ((vel_vector.v * vel_vector.v + vel_vector.w * vel_vector.w) < MIN_VEL_THRESHOLD)
    {
        state_machine.output_velocity.setVelocity(ros_msgs_lw::Twist2D(ros_msgs_lw::Twist2D(0, 0)));

        state_machine.setState(new Idle);
    } 
    else
        state_machine.output_velocity.setVelocity(ros_msgs_lw::Twist2D(vel_vector));
}

void DriveWithVelocity::stop(StateMachine& state_machine)
{
    state_machine.setState(new Idle);

    ros_msgs_lw::Twist2D vel_vector(0, 0);
    state_machine.output_velocity.setVelocity(ros_msgs_lw::Twist2D(vel_vector));
}

void DriveToPoint::set_velocity(StateMachine& state_machine, ros_msgs_lw::Twist2D const& vel_vector)
{
    if ((vel_vector.v * vel_vector.v + vel_vector.w * vel_vector.w) < MIN_VEL_THRESHOLD)
    {
        state_machine.controller_master.stop_controller();

        state_machine.setState(new Idle);
    } 
}

void DriveToPoint::set_goal_point(StateMachine& state_machine, ros_msgs_lw::Point2D const& goal_point)
{
    state_machine.controller_master.start_controller(new p2pController(0.1, 0.1, goal_point), std::bind(&StateMachine::stop,& state_machine));

    state_machine.setState(new DriveToPoint);
}

void DriveToPoint::stop(StateMachine& state_machine)
{
    state_machine.controller_master.stop_controller();

    state_machine.setState(new Idle);
}