#include "approxLinController.h" 

#include "esp_log.h"

#define TAG "approxLinController"

approxLinController::approxLinController(std::shared_ptr<ros_msgs::Trajectory> trajectory) : _trajectory{trajectory} {}

ros_msgs_lw::Twist2D approxLinController::update(ros_msgs_lw::Pose2D const& actual_pose)
{   
    ros_msgs_lw::Twist2D output_vel;

    if(_trajectory_cntr < _trajectory->getTrajectorySize())
    {
        ros_msgs::TrajectoryStateVector setpoint_vector = (*_trajectory)[_trajectory_cntr];
        _trajectory_cntr++;

        ros_msgs::Twist2D setpoint_vel;
        setpoint_vel.v = sqrt(pow(setpoint_vector.dx, 2) + pow(setpoint_vector.dy, 2));
        setpoint_vel.w = (setpoint_vector.ddy * setpoint_vector.dx - setpoint_vector.ddx * setpoint_vector.dy) / (pow(setpoint_vector.dx, 2) + pow(setpoint_vector.dy, 2));

        float setpoint_theta = atan2(setpoint_vector.dy, setpoint_vector.dx);

        float error_1 = cos(actual_pose.theta) * (setpoint_vector.x - actual_pose.x) + sin(actual_pose.theta) * (setpoint_vector.y - actual_pose.y);
        float error_2 = -sin(actual_pose.theta) * (setpoint_vector.x - actual_pose.x) + cos(actual_pose.theta) * (setpoint_vector.y - actual_pose.y);
        float error_3 = setpoint_theta - actual_pose.theta;
        error_3 = atan2(sin(error_3), cos(error_3));

        float k1 = 2 * _damping_coefficient * _natural_frequency;
        float k2 = (pow(_natural_frequency, 2) - pow(setpoint_vel.w, 2)) / setpoint_vel.v;
        float k3 = k1;

        float u1 = - k1 * error_1;
        float u2 = - k2 * error_2 - k3 * error_3;

        output_vel.v = setpoint_vel.v * cos(error_3) - u1;
        output_vel.w = setpoint_vel.w - u2;
    }
    else
    {
        output_vel = 0;
        _destination_reached = true;
    }

    return output_vel;
}

bool approxLinController::destination_reached()
{
    return _destination_reached;
}