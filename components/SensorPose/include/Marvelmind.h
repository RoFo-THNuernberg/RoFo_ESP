#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include "RosMsgs.h"
#include "SensorPose.h"

namespace MARVELMIND
{

    class Marvelmind : public SensorPose 
    {
        public:
            static SensorPose& init();
            ros_msgs::Pose2D get_Pose() override;

        private:
            Marvelmind();
            Marvelmind(Marvelmind const&) = delete;
            ~Marvelmind() {}

            ros_msgs::Pose2D _current_pose;

            static const uart_port_t _uart_port; 
            static const uart_config_t _uart_conf;
    };
}