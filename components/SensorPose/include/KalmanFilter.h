#pragma once

#include <initializer_list>
#include <vector>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "math.h"

#include "SensorPose.h"
#include "KalmanSensor.h"
#include "OutputVelocity.h"
#include "RosMsgsLw.h"
#include "mat.h"

class KalmanFilter : public SensorPose
{
    public:
        //The first sensor in the list must be of absolute type
        static SensorPose& init(std::initializer_list<KalmanSensor const*> const& sensor_list, OutputVelocity const& output_velocity);

        void reInit() override;

    private:
        KalmanFilter(std::initializer_list<KalmanSensor const*> const& sensor_list, OutputVelocity const& output_velocity);
        KalmanFilter(KalmanFilter const&) = delete;
        ~KalmanFilter();

        static void _kalman_filter_loop_timer(TimerHandle_t timer);
        static void _kalman_filter_loop_task(void* pvParameters);

        static KalmanFilter* _kalman_filter;

        //The first sensor in the vector must be of absolute type
        std::vector<KalmanSensor const*> _sensor_list;

        OutputVelocity const& _output_velocity;

        ros_msgs_lw::Pose2D _a_posterior_estimate;
        dspm::Mat _a_posterior_cov;

        float _process_noise_variance = 0.001;
        dspm::Mat _process_noise_cov;

        uint64_t _timestamp_us;

        SemaphoreHandle_t _reinitialize_sensor_semphr;

        TimerHandle_t _kalman_filter_loop_timer_handle;
        TaskHandle_t _kalman_filter_loop_task_handle;
};