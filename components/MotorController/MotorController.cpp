#include "MotorController.h"


#define TIMER_DIVIDER 16
#define TIMER_TICKS_PER_US (TIMER_BASE_CLK / TIMER_DIVIDER / 1000000)
#define TIMER_PERIOD_US 1000

MotorController* MotorController::_motor_controller_a = nullptr;
MotorController* MotorController::_motor_controller_b = nullptr;

MotorController::MotorController(Motor& motor, bool motor_dir, timer_idx_t timer_index) : _motor{motor}, _motor_dir{motor_dir}, _timer_index{timer_index}
{
    _timer_config.divider = TIMER_DIVIDER;                            
    _timer_config.counter_dir = TIMER_COUNT_UP;                      
    _timer_config.counter_en = TIMER_PAUSE;                            
    _timer_config.alarm_en = TIMER_ALARM_EN;                          
    _timer_config.auto_reload = TIMER_AUTORELOAD_EN;
    _timer_config.intr_type = TIMER_INTR_LEVEL;
    _timer_config.clk_src = TIMER_SRC_CLK_APB;

    _kp = 50;
    _ki = 500.0;

    _prev_time_us = esp_timer_get_time();
}

void MotorController::_init()
{
    xTaskCreate(_motor_control_loop_task, "_motor_control_loop_task", 2048, this, 10, &_control_loop_task);

    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, _timer_index, &_timer_config));

    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, _timer_index, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, _timer_index, TIMER_PERIOD_US * TIMER_TICKS_PER_US));

    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, _timer_index));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, _timer_index, _motor_control_notify, &_control_loop_task, ESP_INTR_FLAG_IRAM));

    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, _timer_index));
}

MotorController& MotorController::getMotorControllerA()
{
    if(_motor_controller_a == nullptr)
    {
        _motor_controller_a = new MotorController(Motor::getMotorA(), true, TIMER_0);

        _motor_controller_a->_init();
    }

    return *_motor_controller_a;
}

MotorController& MotorController::getMotorControllerB()
{
    if(_motor_controller_b == nullptr)
    {
        _motor_controller_b = new MotorController(Motor::getMotorB(), false, TIMER_1);

        _motor_controller_b->_init();
    }

    return *_motor_controller_b;
}

void MotorController::setVelocity(float setpoint_velocity)
{
    _setpoint_velocity = setpoint_velocity;
}


void MotorController::_motor_control_loop_task(void* pvParameters)
{
    MotorController& motor_controller = *(MotorController*)pvParameters;

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //Input
        float actual_velocity = motor_controller._motor.getVelocity();
        motor_controller.actual_velocity = actual_velocity;

        //Calculate Error
        float error = 0;
        if(motor_controller._motor_dir)
            error = actual_velocity - motor_controller._setpoint_velocity;
        else
            error = motor_controller._setpoint_velocity - actual_velocity;

        //Calculate Proportional Output
        float p_out = motor_controller._kp * error;

        //Calculate Integral Output
        int64_t current_time_us = esp_timer_get_time();
        int64_t delta_time_us = current_time_us - motor_controller._prev_time_us;
        motor_controller.loop_time_us = delta_time_us;
        motor_controller._prev_time_us = current_time_us;

        motor_controller._error_integral += error * (float)delta_time_us / 1000000;

        float i_out = motor_controller._ki * motor_controller._error_integral;

        if(i_out > 100)
            motor_controller._error_integral = 100 / motor_controller._ki;
        else if (i_out < -100)
            motor_controller._error_integral = -100 / motor_controller._ki;


        //Calculate Ouput
        float output_duty_cycle = p_out + i_out;
        motor_controller.output_duty = output_duty_cycle;

        //Output
        motor_controller._motor.setDuty(output_duty_cycle);
    }

    vTaskDelete(NULL);
}


bool IRAM_ATTR MotorController::_motor_control_notify(void* args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TaskHandle_t* control_loop_task = (TaskHandle_t*)args;

    vTaskNotifyGiveFromISR(*control_loop_task, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

