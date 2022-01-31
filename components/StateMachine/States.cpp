#include "States.h"
#include "StateMachine.h"

#define MIN_VEL_THRESHOLD 0.01

std::string const Idle::_state = "Idle";
std::string const DriveWithVelocity::_state = "DriveWithVelocity";
std::string const DriveToPoint::_state = "DriveToPoint";


void Idle::set_velocity(StateMachine& state_machine, std::shared_ptr<ros_msgs::Twist2D> vel_vector_msg)
{
    ros_msgs_lw::Twist2D vel_vector(*vel_vector_msg);
    
    state_machine.output_velocity.setVelocity(vel_vector);

    if (vel_vector.v != 0 || vel_vector.w != 0)
        state_machine.setState(new DriveWithVelocity);
    
}

void Idle::set_goal_point(StateMachine& state_machine, std::shared_ptr<ros_msgs::Point2D> goal_point)
{
    state_machine.controller_master.start_controller(new p2pController(0.2, 0.8, ros_msgs_lw::Point2D(*goal_point)), std::bind(&StateMachine::stop,& state_machine));

    state_machine.setState(new DriveToPoint);
}

void DriveWithVelocity::set_velocity(StateMachine& state_machine, std::shared_ptr<ros_msgs::Twist2D> vel_vector_msg)
{
    
    ros_msgs_lw::Twist2D vel_vector(*vel_vector_msg);
    
    if (vel_vector.v == 0 && vel_vector.w == 0)
        state_machine.setState(new Idle);

    state_machine.output_velocity.setVelocity(vel_vector);
    
}

void DriveWithVelocity::stop(StateMachine& state_machine)
{
    state_machine.setState(new Idle);

    ros_msgs_lw::Twist2D vel_vector(0, 0);
    state_machine.output_velocity.setVelocity(vel_vector);
}

void DriveToPoint::set_velocity(StateMachine& state_machine, std::shared_ptr<ros_msgs::Twist2D> vel_vector_msg)
{
    ros_msgs_lw::Twist2D vel_vector(*vel_vector_msg);

    if (vel_vector.v == 0 && vel_vector.w == 0)
    {
        state_machine.controller_master.stop_controller();

        state_machine.setState(new Idle);
    } 
}

void DriveToPoint::set_goal_point(StateMachine& state_machine, std::shared_ptr<ros_msgs::Point2D> goal_point)
{
    state_machine.controller_master.start_controller(new p2pController(0.1, 0.1, ros_msgs_lw::Point2D(*goal_point)), std::bind(&StateMachine::stop,& state_machine));

    state_machine.setState(new DriveToPoint);
}

void DriveToPoint::stop(StateMachine& state_machine)
{
    state_machine.controller_master.stop_controller();

    state_machine.setState(new Idle);
}