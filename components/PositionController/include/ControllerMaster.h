#pragma once

#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "PositionController.h"
#include "RosMsgs.h"
#include "OutputVelocity.h"
#include "SensorPose.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_err.h"


class ControllerMaster
{
    public:
        static void start_controller(PositionController* pos_controller, std::function<void()>callback_function);
        static void stop_controller();

    private:
        ControllerMaster();
        ControllerMaster(const ControllerMaster&) = delete;
        ~ControllerMaster();

        static void _control_loop(TimerHandle_t);

        static ControllerMaster* _controller_obj;

        PositionController* _pos_controller;
        static SemaphoreHandle_t _pos_controller_mutx;

        std::function<void()>_callback_function;

        OutputVelocity& _output_velocity;
        SensorPose& _sensor_pose;

        static TimerHandle_t _control_loop_timer_handle;
        bool _timer_is_stopped = true;
};
