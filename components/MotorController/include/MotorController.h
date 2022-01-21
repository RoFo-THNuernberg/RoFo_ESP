#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Motor.h"


class MotorController
{
    public:
        static MotorController& init();

        void setVelocity(float setpoint_velocity_a, float setpoint_velocity_b);

    private:
        MotorController();
        MotorController(MotorController const&) = delete;
        ~MotorController() {};

        static void _motor_control_loop_task(void* pvParameters);
        static bool IRAM_ATTR _motor_control_interrupt(void* args);

        TaskHandle_t _control_loop_task;

        static MotorController* _motor_controller;

        Motor& _motor_a;
        Motor& _motor_b;

        static gpio_config_t _enable_config;
        gpio_num_t _enable_pin;

        static timer_config_t _timer_config;
};