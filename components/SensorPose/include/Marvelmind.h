#pragma once

#include <array>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/uart.h"

#include "math.h"

#include "RosMsgsLw.h"
#include "SensorPose.h"
#include "KalmanSensor.h"
#include "mat.h"

class Marvelmind : public KalmanSensor, public SensorPose
{
    public:
        static Marvelmind& init();

        void reInit() override {}

        bool calculateKalman(ros_msgs_lw::Pose2D const& a_priori_estimate, dspm::Mat const& a_priori_cov, ros_msgs_lw::Pose2D& a_posterior_estimate, dspm::Mat& a_posterior_cov) const override;
        bool getInitialPose(ros_msgs_lw::Pose2D& initial_pose) const override;
        void getMeasurementNoiseCov(dspm::Mat& measurement_cov) const override;

    private:
        Marvelmind();
        Marvelmind(Marvelmind const&) = delete;
        ~Marvelmind();

        static void _uart_read_data_task(void* pvParameters);

        static Marvelmind* _marvelmind_sensor;

        QueueHandle_t _current_pose_queue;

        dspm::Mat const _measurement_noise_cov;

        static const uart_port_t _uart_port; 
        static const uart_config_t _uart_conf;

        TaskHandle_t _uart_read_data_task_handle;
};