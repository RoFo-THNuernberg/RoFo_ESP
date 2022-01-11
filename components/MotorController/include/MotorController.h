#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "esp_timer.h"

#include "Motor.h"


class MotorController
{
    public:
        static MotorController& getMotorControllerA();
        static MotorController& getMotorControllerB();

        void setVelocity(float setpoint_velocity);

        int64_t loop_time_us;
        float actual_velocity;
        float output_duty;

    private:
        MotorController(Motor& motor, bool motor_dir, timer_idx_t timer_index);
        MotorController(MotorController const&) = delete;
        ~MotorController() {};

        void _init();

        static void _motor_control_loop_task(void* pvParameters);
        //static bool IRAM_ATTR _motor_control_loop(void* args);
        static bool IRAM_ATTR _motor_control_notify(void* args);

        TaskHandle_t _control_loop_task;

        static MotorController* _motor_controller_a;
        static MotorController* _motor_controller_b;

        Motor& _motor;

        float _kp, _ki;

        bool _motor_dir;

        float _setpoint_velocity = 0;
        int64_t _prev_time_us;
        float _error_integral = 0;


        timer_config_t _timer_config;
        timer_idx_t _timer_index;
};