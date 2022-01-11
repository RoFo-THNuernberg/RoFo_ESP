#pragma once

#include <array>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include "RosMsgs.h"
#include "SensorPose.h"
#include "KalmanSensor.h"
#include "matrix_math.h"

class Marvelmind : public SensorPose, public KalmanSensor
{
    public:
        static SensorPose& init();
        ros_msgs::Pose2D getPose() override;

        bool newData() override;
        //void calculateKalmanGain(std::array<std::array<float, 3>, 3> const& a_priori_cov) override;
        //ros_msgs::Pose2D getPosterioriPose(ros_msgs::Pose2D const& a_priori_pose) override;
        //std::array<std::array<float, 3>, 3> getPosterioriCov(std::array<std::array<float, 3>, 3> const&) override;

    private:
        Marvelmind();
        Marvelmind(Marvelmind const&) = delete;
        ~Marvelmind() {}

        static void _uart_read_data_task(void* pvParameters);

        static Marvelmind* _marvelmind_sensor;

        ros_msgs::Pose2D _current_pose;

        bool _new_data_ready = false;

        std::array<std::array<float, 3>, 3> const _measurement_noise_cov;
        std::array<std::array<float, 3>, 3> _kalman_gain;

        static const uart_port_t _uart_port; 
        static const uart_config_t _uart_conf;
};