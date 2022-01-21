#include "statInOutLinController.h"

statInOutLinController::statInOutLinController(float kp_1, float kp_2, float b_offset, ros_msgs::Trajectory const& trajectory) : 
    _kp_1{kp_1}, _kp_2{kp_2}, _b_offset{b_offset}, _trajectory{trajectory} {}

ros_msgs_lw::Twist2D statInOutLinController::update(ros_msgs_lw::Pose2D const& actual_pose) 
{   
    ros_msgs_lw::Twist2D output_vel;

    if(_trajectory_cntr < _trajectory.getTrajectorySize())
    {
        ros_msgs::TrajectoryStateVector const&  setpoint_vector = _trajectory[_trajectory_cntr];
        _trajectory_cntr++;

        //calculate current position of point b
        ros_msgs::Pose2D actual_b_pose;
        actual_b_pose.x = actual_pose.x + _b_offset * cos(actual_pose.theta);
        actual_b_pose.y = actual_pose.y + _b_offset * sin(actual_pose.theta);

        float u1 = setpoint_vector.dx + _kp_1 * (setpoint_vector.x - actual_b_pose.x);
        float u2 = setpoint_vector.dy + _kp_2 * (setpoint_vector.y - actual_b_pose.y);

        output_vel.v = u1 * cos(actual_pose.theta) + u2 * sin(actual_pose.theta);
        output_vel.w = - u1 * sin(actual_pose.theta) / _b_offset + u2 * cos(actual_pose.theta) / _b_offset;
    }
    else
    {
        output_vel = 0;
        _destination_reached = true;
    }

    return output_vel;
}

bool statInOutLinController::destination_reached()
{
    return _destination_reached;
}