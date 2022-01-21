#pragma once

#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "OutputVelocity.h"
#include "SensorPose.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"


class ControllerMaster
{
    public:
        void start_controller(PositionController* pos_controller, std::function<void()>destination_reached_callback);
        void stop_controller();

        static ControllerMaster& init(OutputVelocity& output_velocity, SensorPose& sensor_pose);

    private:
        ControllerMaster(OutputVelocity& ouput_velocity, SensorPose& sensor_pose);
        ControllerMaster(const ControllerMaster&) = delete;
        ~ControllerMaster();

        static void _control_loop_timer(TimerHandle_t timer);
        static void _control_loop_task(void* pvParameters);

        static ControllerMaster* _controller_obj;

        uint64_t prev_time = 0;

        PositionController* _pos_controller;
        SemaphoreHandle_t _pos_controller_mutx;

        std::function<void()>_destination_reached_callback;

        OutputVelocity& _output_velocity;
        SensorPose& _sensor_pose;

        TimerHandle_t _control_loop_timer_handle;
        TaskHandle_t _control_loop_task_handle;

        bool _timer_is_stopped;
};
