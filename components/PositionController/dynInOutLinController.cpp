#include "dynInOutLinController.h"

dynInOutLinController::dynInOutLinController(float kp_1, float kp_2, float kd_1, float kd_2, ros_msgs::Trajectory const& trajectory) : 
    _kp_1{kp_1}, _kp_2{kp_2}, _kd_1{kd_1}, _kd_2{kd_2}, _prev_time_us{(uint64_t)esp_timer_get_time()}, _prev_velocity{0.0001}, _prev_acceleration{0}, _trajectory{trajectory} {}

ros_msgs::Twist2D dynInOutLinController::update(ros_msgs::Pose2D const& actual_pose)
{   
    ros_msgs::Twist2D output_vel;

    if(_trajectory_cntr < _trajectory.getTrajectorySize())
    {
        ros_msgs::TrajectoryStateVector const&  setpoint_vector = _trajectory[_trajectory_cntr];
        _trajectory_cntr++;

        /*calculate time since last excecution*/
        uint64_t current_time_us = esp_timer_get_time();
        uint64_t delta_time_us = current_time_us - _prev_time_us;
        _prev_time_us = current_time_us;

        float actual_dx = (actual_pose.x - _prev_pose.x) / delta_time_us * 1000000.0;
        float actual_dy = (actual_pose.y - _prev_pose.y) / delta_time_us * 1000000.0;

        float u1 = setpoint_vector.ddx + _kp_1 * (setpoint_vector.x - actual_pose.x) + _kd_1 * (setpoint_vector.dx - actual_dx);
        float u2 = setpoint_vector.ddy + _kp_2 * (setpoint_vector.y - actual_pose.y) + _kd_2 * (setpoint_vector.dy - actual_dy);

        float output_acceleration = u1 * cos(actual_pose.theta) + u2 * sin(actual_pose.theta);
        output_vel.w = - u1 * sin(actual_pose.theta) / _prev_velocity + u2 * cos(actual_pose.theta) / _prev_velocity;

        _prev_velocity = delta_time_us / 1000000.0 * (_prev_acceleration + output_acceleration) / 2 + _prev_velocity;
        _prev_acceleration = output_acceleration;
        
        output_vel.v = _prev_velocity;
    }
    else
    {
        output_vel = 0;
        _destination_reached = true;
    }

    return output_vel;
}

bool dynInOutLinController::destination_reached()
{
    return _destination_reached;
}