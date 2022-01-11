#include "KalmanFilter.h"

#define MAX_DELTA_TIME 100000

KalmanFilter* KalmanFilter::_kalman_filter = nullptr;

KalmanFilter::KalmanFilter(const std::initializer_list<KalmanSensor*>& sensor_list) : _sensor_list(sensor_list), 
    _process_noise_cov{0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001} {}

SensorPose& KalmanFilter::init(const std::initializer_list<KalmanSensor*>& sensor_list)
{
    if(_kalman_filter == nullptr)
        _kalman_filter = new KalmanFilter(sensor_list);

    return *_kalman_filter;
}

ros_msgs::Pose2D KalmanFilter::getPose() 
{   

    uint64_t current_timestamp_us = esp_timer_get_time();

    uint64_t delta_time = current_timestamp_us - _timestamp_us;

    if(delta_time < MAX_DELTA_TIME)
    {
        delta_time = 0;
    }

    ros_msgs::Twist2D cmd_vel = OutputVelocity::getOutput().getVelocity();

    //linearize transition function
    std::array<std::array<float, 3>, 3> transition_lin;
    transition_lin[0][0] = 1;
    transition_lin[0][2] = -cmd_vel.v * sin(_a_posterior_estimate.theta) * delta_time; 
    transition_lin[1][1] = 1;
    transition_lin[1][2] = cmd_vel.v * cos(_a_posterior_estimate.theta) * delta_time; 
    transition_lin[2][2] = 1;

    std::array<std::array<float, 3>, 3> transition_lin_tranpose;
    transition_lin_tranpose[0][0] = 1;
    transition_lin_tranpose[1][1] = 1;
    transition_lin_tranpose[2][0] = -cmd_vel.v * sin(_a_posterior_estimate.theta) * delta_time; 
    transition_lin_tranpose[2][1] = cmd_vel.v * cos(_a_posterior_estimate.theta) * delta_time; 
    transition_lin_tranpose[2][2] = 1;

    //Predict
    std::array<float, 3> delta_pose;
    delta_pose[0] = cmd_vel.v * cos(_a_posterior_estimate.theta) * delta_time;
    delta_pose[1] = cmd_vel.v * sin(_a_posterior_estimate.theta) * delta_time;
    delta_pose[2] = cmd_vel.w * delta_time;

    ros_msgs::Pose2D a_priori_estimate;
    a_priori_estimate = _a_posterior_estimate + delta_pose;

    std::array<std::array<float, 3>, 3> a_priori_cov;
    a_priori_cov = transition_lin * _a_posterior_cov * transition_lin_tranpose + _process_noise_cov;

    //Update
    for(auto i : _sensor_list)
    {
        if(i->newData() == true)
        {

        }
    }

    return a_priori_estimate;
}