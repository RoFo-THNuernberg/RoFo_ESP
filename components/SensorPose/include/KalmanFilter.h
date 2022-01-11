#pragma once

#include <initializer_list>
#include <vector>
#include <array>

#include "esp_timer.h"
#include "math.h"

#include "SensorPose.h"
#include "KalmanSensor.h"
#include "OutputVelocity.h"
#include "RosMsgs.h"
#include "matrix_math.h"

class KalmanFilter : public SensorPose
{
    public:
        static SensorPose& init(const std::initializer_list<KalmanSensor*>& sensor_list);
        ros_msgs::Pose2D getPose() override;

    private:
        KalmanFilter(const std::initializer_list<KalmanSensor*>& sensor_list);
        KalmanFilter(KalmanFilter const&) = delete;
        ~KalmanFilter() {}

        static KalmanFilter* _kalman_filter;

        std::vector<KalmanSensor*> _sensor_list;

        ros_msgs::Pose2D _a_posterior_estimate;
        std::array<std::array<float, 3>, 3> _a_posterior_cov;

        std::array<std::array<float, 3>, 3> const _process_noise_cov;

        uint64_t _timestamp_us;

};