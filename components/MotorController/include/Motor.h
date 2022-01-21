#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "soc/rtc.h"

class Motor
{
    public:
        static Motor& getMotorA();
        static Motor& getMotorB();

        void setVelocity(float setpoint_velocity);
        float getActualVelocity();

        void updatePIControl();

    private:
        Motor(mcpwm_unit_t mcpwm_unit, mcpwm_pin_config_t motor_pins, bool motor_dir);
        ~Motor() {}
        Motor(Motor const&) = delete;

        void _setDuty(float duty_cycle);

        static bool IRAM_ATTR _encoder_callback(mcpwm_unit_t mcpwm_unit, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata, void *user_data);

        static Motor* _motor_a;
        static Motor* _motor_b;

        float _kp, _ki;

        bool _motor_dir;

        float _setpoint_velocity = 0;

        int64_t _prev_time_us = 0;
        float _error_integral = 0;

        uint32_t _encoder_timestamp;

        int32_t _encoder_pulse_period;
        int32_t _prev_pulse_period;

        float _current_duty_cycle;

        mcpwm_unit_t _mcpwm_unit;
        mcpwm_timer_t _mcpwm_timer;
        mcpwm_pin_config_t _motor_pins;
        mcpwm_config_t _mcpwm_config;

        mcpwm_capture_config_t _mcpwm_capture_config_0;
        mcpwm_capture_config_t _mcpwm_capture_config_1;
};