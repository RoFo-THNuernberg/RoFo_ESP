#include "Motor.h"

#define MOTOR_A_IN1 CONFIG_MOTOR_A_IN1
#define MOTOR_A_IN2 CONFIG_MOTOR_A_IN2
#define MOTOR_A_INCREMENTAL_ENCODER_1 CONFIG_MOTOR_A_INCREMENTAL_ENCODER_1
#define MOTOR_A_INCREMENTAL_ENCODER_2 CONFIG_MOTOR_A_INCREMENTAL_ENCODER_2

#define MOTOR_B_IN1 CONFIG_MOTOR_B_IN1
#define MOTOR_B_IN2 CONFIG_MOTOR_B_IN2
#define MOTOR_B_INCREMENTAL_ENCODER_1 CONFIG_MOTOR_B_INCREMENTAL_ENCODER_1
#define MOTOR_B_INCREMENTAL_ENCODER_2 CONFIG_MOTOR_B_INCREMENTAL_ENCODER_2

#define MOTOR_PWM_FREQUENCY CONFIG_MOTOR_PWM_FREQUENCY

#define TICK_PER_S_TO_ROTATION_PER_S (1. / (10 * 63))


Motor* Motor::_motor_a = nullptr;
Motor* Motor::_motor_b = nullptr;

Motor::Motor(mcpwm_unit_t mcpwm_unit, mcpwm_pin_config_t motor_pins) : _mcpwm_unit{mcpwm_unit}, _motor_pins{motor_pins}
{
    _mcpwm_timer = MCPWM_TIMER_0;
    _mcpwm_config = (mcpwm_config_t){ .frequency = MOTOR_PWM_FREQUENCY, .cmpr_a = 0, .cmpr_b = 0, .duty_mode = MCPWM_DUTY_MODE_0, .counter_mode = MCPWM_UP_COUNTER};
    _mcpwm_capture_config_0 = (mcpwm_capture_config_t) {.cap_edge = MCPWM_POS_EDGE, .cap_prescale = 1, .capture_cb = _encoder_callback, .user_data = this};
    _mcpwm_capture_config_1 = (mcpwm_capture_config_t) {.cap_edge = MCPWM_BOTH_EDGE, .cap_prescale = 1, .capture_cb = NULL, .user_data = NULL};
}

Motor& Motor::getMotorA()
{
    if(_motor_a == nullptr)
    {

        mcpwm_pin_config_t motor_pins =
        {
            .mcpwm0a_out_num = MOTOR_A_IN1,    
            .mcpwm0b_out_num = MOTOR_A_IN2,    
            .mcpwm1a_out_num = -1,    
            .mcpwm1b_out_num = -1,    
            .mcpwm2a_out_num = -1,    
            .mcpwm2b_out_num = -1,    
            .mcpwm_sync0_in_num = -1, 
            .mcpwm_sync1_in_num = -1, 
            .mcpwm_sync2_in_num = -1, 
            .mcpwm_fault0_in_num = -1,
            .mcpwm_fault1_in_num = -1,
            .mcpwm_fault2_in_num = -1,
            .mcpwm_cap0_in_num = MOTOR_A_INCREMENTAL_ENCODER_1,  
            .mcpwm_cap1_in_num = MOTOR_A_INCREMENTAL_ENCODER_2,  
            .mcpwm_cap2_in_num = -1,  
        };


        _motor_a = new Motor(MCPWM_UNIT_0, motor_pins);

        _motor_a->_init();
    }
        
    return *_motor_a;
}

Motor& Motor::getMotorB()
{
    if(_motor_b == nullptr)
    {
        mcpwm_pin_config_t motor_pins =
        {
            .mcpwm0a_out_num = MOTOR_B_IN1,    
            .mcpwm0b_out_num = MOTOR_B_IN2,    
            .mcpwm1a_out_num = -1,    
            .mcpwm1b_out_num = -1,    
            .mcpwm2a_out_num = -1,    
            .mcpwm2b_out_num = -1,    
            .mcpwm_sync0_in_num = -1, 
            .mcpwm_sync1_in_num = -1, 
            .mcpwm_sync2_in_num = -1, 
            .mcpwm_fault0_in_num = -1,
            .mcpwm_fault1_in_num = -1,
            .mcpwm_fault2_in_num = -1,
            .mcpwm_cap0_in_num = MOTOR_B_INCREMENTAL_ENCODER_1,  
            .mcpwm_cap1_in_num = MOTOR_B_INCREMENTAL_ENCODER_2,  
            .mcpwm_cap2_in_num = -1,  
        };
        
        _motor_b = new Motor(MCPWM_UNIT_1, motor_pins);

        _motor_b->_init();
    }
        
    return *_motor_b;
}

void Motor::_init()
{
    ESP_ERROR_CHECK(mcpwm_set_pin(_mcpwm_unit, &_motor_pins));
    ESP_ERROR_CHECK(mcpwm_init(_mcpwm_unit, _mcpwm_timer, &_mcpwm_config));
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(_mcpwm_unit, MCPWM_SELECT_CAP0, &_mcpwm_capture_config_0));
    ESP_ERROR_CHECK(mcpwm_capture_enable_channel(_mcpwm_unit, MCPWM_SELECT_CAP1, &_mcpwm_capture_config_1));
}

void Motor::setDuty(float duty_cycle)
{   
    _current_duty_cycle = duty_cycle;

    //mcpwm_set_duty_in_us() might not be safe to be called from an interrupt!?
    if(duty_cycle > 0)
    {
        mcpwm_set_duty(_mcpwm_unit, _mcpwm_timer, MCPWM_GEN_A, 0);
        mcpwm_set_duty(_mcpwm_unit, _mcpwm_timer, MCPWM_GEN_B, duty_cycle);
    } 
    else if(duty_cycle < 0)
    {
        mcpwm_set_duty(_mcpwm_unit, _mcpwm_timer, MCPWM_GEN_A, -duty_cycle);
        mcpwm_set_duty(_mcpwm_unit, _mcpwm_timer, MCPWM_GEN_B, 0);
    }
    else
    {   
        _current_duty_cycle = 0;
        mcpwm_set_duty(_mcpwm_unit, _mcpwm_timer, MCPWM_GEN_A, 0);
        mcpwm_set_duty(_mcpwm_unit, _mcpwm_timer, MCPWM_GEN_B, 0);
    }
}

float Motor::getVelocity()
{
    if(_current_duty_cycle == 0 && _prev_pulse_period == _encoder_pulse_period)
        _encoder_pulse_period = 0;

    _prev_pulse_period = _encoder_pulse_period;

    return (_encoder_pulse_period != 0) ? ((float)rtc_clk_apb_freq_get() / _encoder_pulse_period) * TICK_PER_S_TO_ROTATION_PER_S : 0;
}

bool IRAM_ATTR Motor::_encoder_callback(mcpwm_unit_t mcpwm_unit, mcpwm_capture_channel_id_t cap_channel, const cap_event_data_t *edata, void *user_data)
{
    Motor* motor = (Motor*)user_data;

    mcpwm_capture_on_edge_t cap_channel_1_state = (mcpwm_capture_on_edge_t)mcpwm_capture_signal_get_edge(mcpwm_unit, MCPWM_SELECT_CAP1);

    if(cap_channel_1_state == MCPWM_POS_EDGE)
        motor->_encoder_pulse_period = edata->cap_value - motor->_encoder_timestamp;
    else if(cap_channel_1_state == MCPWM_NEG_EDGE)
        motor->_encoder_pulse_period = motor->_encoder_timestamp - edata->cap_value;

    motor->_encoder_timestamp = edata->cap_value;

    return pdFALSE;
}