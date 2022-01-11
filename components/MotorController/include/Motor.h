#pragma once

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include "soc/rtc.h"

class Motor
{
    public:
        static Motor& getMotorA();
        static Motor& getMotorB();

        void setDuty(float duty_cycle);
        float getVelocity();

    private:
        Motor(mcpwm_unit_t mcpwm_unit, mcpwm_pin_config_t motor_pins);
        Motor(Motor const&) = delete;
        ~Motor() {}

        void _init();

        static bool IRAM_ATTR _encoder_callback(mcpwm_unit_t mcpwm_unit, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata, void *user_data);

        static Motor* _motor_a;
        static Motor* _motor_b;

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