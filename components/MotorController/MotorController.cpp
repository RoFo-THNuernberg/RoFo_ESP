#include "MotorController.h"


#define TIMER_DIVIDER 16
#define TIMER_TICKS_PER_US (TIMER_BASE_CLK / TIMER_DIVIDER / 1000000)
#define TIMER_PERIOD_US 1000

#define MAX_MOTOR_RPS (220 / 60)

MotorController* MotorController::_motor_controller = nullptr;
timer_config_t MotorController::_timer_config = 
{
    .alarm_en = TIMER_ALARM_EN,
    .counter_en = TIMER_PAUSE,
    .intr_type = TIMER_INTR_LEVEL,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .clk_src = TIMER_SRC_CLK_APB,
    .divider = TIMER_DIVIDER
};
gpio_config_t MotorController::_enable_config = 
{
    .pin_bit_mask = GPIO_SEL_23,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

MotorController::MotorController() : _motor_a(Motor::getMotorA()), _motor_b(Motor::getMotorB()) 
{
    xTaskCreate(_motor_control_loop_task, "_motor_control_loop_task", 2048, this, 10, &_control_loop_task);

    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &_timer_config));

    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_PERIOD_US * TIMER_TICKS_PER_US));

    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, _motor_control_interrupt, &_control_loop_task, ESP_INTR_FLAG_IRAM));

    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));

    _enable_pin = GPIO_NUM_23;

    ESP_ERROR_CHECK(gpio_config(&_enable_config));
    gpio_set_level(_enable_pin, 1);
}

MotorController& MotorController::init()
{
    if(_motor_controller == nullptr)
        _motor_controller = new MotorController();

    return *_motor_controller;
}

void MotorController::setVelocity(float setpoint_velocity_a, float setpoint_velocity_b)
{
    if(abs(setpoint_velocity_a) < MAX_MOTOR_RPS && abs(setpoint_velocity_b) < MAX_MOTOR_RPS)
    {
        _motor_a.setVelocity(setpoint_velocity_a);
        _motor_b.setVelocity(setpoint_velocity_b);
    }
    
}

void MotorController::_motor_control_loop_task(void* pvParameters)
{
    MotorController& motor_controller = *(MotorController*)pvParameters;

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        motor_controller._motor_a.updatePIControl();
        motor_controller._motor_b.updatePIControl();              
    }

    vTaskDelete(NULL);
}

bool IRAM_ATTR MotorController::_motor_control_interrupt(void* args)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    TaskHandle_t* control_loop_task = (TaskHandle_t*)args;

    vTaskNotifyGiveFromISR(*control_loop_task, &xHigherPriorityTaskWoken);

    return xHigherPriorityTaskWoken;
}

