#pragma once 

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "SensorPose.h"
#include "RosMsgsLw.h"
#include "mat.h"

class KalmanSensor
{
    public:
        KalmanSensor();
        ~KalmanSensor();

        //calculation of KalmanGain is located in the corresponding KalmanSensor in order to optimize processing time and memory usage 
        virtual bool calculateKalman(ros_msgs_lw::Pose2D const& a_priori_estimate, dspm::Mat const& a_priori_cov, ros_msgs_lw::Pose2D& a_posterior_estimate, dspm::Mat& a_posterior_cov) const = 0;

        //Implement function only for absolute sensors
        virtual bool getInitialPose(ros_msgs_lw::Pose2D& initial_pose) const = 0;

        //Implement function only for absolute sensors
        virtual void getMeasurementNoiseCov(dspm::Mat& measurement_cov) const = 0;

};