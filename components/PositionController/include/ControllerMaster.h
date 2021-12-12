#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"


class ControllerMaster
{
    public:
        static ControllerMaster& init();

    private:
        ControllerMaster();
        ControllerMaster(const ControllerMaster&) = delete;
        ~ControllerMaster();

        static void _control_loop(TimerHandle_t);

        static ControllerMaster& _controller_obj;

        TimerHandle_t _control_loop_handle;
};
