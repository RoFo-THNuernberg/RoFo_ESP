#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/uart.h"

#include "data_types.h"
#include "SensorPose.h"

namespace MARVELMIND
{

    class Marvelmind : public SensorPose {
        public:
            esp_err_t init() override;
            const data_types::Pose& get_Pose() override;

        private:
            static void read_new_data(void *arg);

            data_types::Pose _current_pose;

            static const uart_port_t _uart_port; 
            static const uart_config_t _uart_conf;
            static QueueHandle_t _pose_queue;
    };
}