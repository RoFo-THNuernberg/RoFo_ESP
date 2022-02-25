#pragma once

#include <functional>
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "RosMsgsLw.h"
#include "RosMsgs.h"
#include "SensorPose.h"
#include "KalmanSensor.h"
#include "NodeHandle.h"

class SensorPoseSim : public SensorPose, public KalmanSensor
{
    public:
        static SensorPoseSim& init(ros::NodeHandle& node_handle);

        
        bool peekAtPose(ros_msgs_lw::Pose2D& current_pose) const override;
        bool getPose(ros_msgs_lw::Pose2D& current_pose) const override;
        void reInit() override {}

        void calculateMeasurementNoiseCov() const override {}
        bool calculateKalman(ros_msgs_lw::Pose2D const& a_priori_estimate, dspm::Mat const& a_priori_cov, ros_msgs_lw::Pose2D& a_posterior_estimate, dspm::Mat& a_posterior_cov) const override;
        bool getAbsolutePose(ros_msgs_lw::Pose2D& initial_pose) const override;
        void getMeasurementNoiseCov(dspm::Mat& measurement_cov) const override;

    private:
        SensorPoseSim(ros::NodeHandle& node_handle);
        SensorPoseSim(SensorPoseSim const&) = delete;
        ~SensorPoseSim();

        xQueueHandle _current_pose_queue;

        dspm::Mat _measurement_noise_cov;

        void _setPose(std::shared_ptr<ros_msgs::Pose2DSim> pose);

        static SensorPoseSim* _sensor_pose_sim;

};

