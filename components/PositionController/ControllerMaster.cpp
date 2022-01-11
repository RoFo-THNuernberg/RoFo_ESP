#include "ControllerMaster.h"

ControllerMaster* ControllerMaster::_controller_obj = nullptr;
//SemaphoreHandle_t ControllerMaster::_pos_controller_mutx{};
//TimerHandle_t ControllerMaster::_control_loop_timer_handle{};

ControllerMaster::ControllerMaster() : _pos_controller{nullptr}, _output_velocity{OutputVelocity::getOutput()}, _sensor_pose{SensorPose::getGlobalSensor()}
{   
    _pos_controller_mutx = xSemaphoreCreateMutex();
    xSemaphoreGive(_pos_controller_mutx);

    /*maybe adjust freeRtos tick rate*/
    _control_loop_timer_handle = xTimerCreate("control_loop", pdMS_TO_TICKS(10), pdTRUE, NULL, _control_loop);

    _timer_is_stopped = true;
    xTimerStop(_control_loop_timer_handle, portMAX_DELAY);
}

ControllerMaster::~ControllerMaster() 
{
    if(_pos_controller != nullptr)
        delete _pos_controller;
    
    _pos_controller = nullptr; 
}

ControllerMaster& ControllerMaster::getControllerMaster()
{
    if(_controller_obj == nullptr)
        _controller_obj = new ControllerMaster();

    return *_controller_obj;
}

void ControllerMaster::start_controller(PositionController* pos_controller, std::function<void()>callback_function)
{   
    if(xSemaphoreTake(_pos_controller_mutx, portMAX_DELAY) == pdPASS)
    {
        if(_controller_obj->_pos_controller != nullptr)
            delete _controller_obj->_pos_controller;

        _controller_obj->_pos_controller = pos_controller;

        _controller_obj->_destination_reached_callback = callback_function;

        _controller_obj->_timer_is_stopped = false;
        xTimerStart(_control_loop_timer_handle, portMAX_DELAY);

        xSemaphoreGive(_pos_controller_mutx);
    }
}

void ControllerMaster::stop_controller()
{   
    _controller_obj->_timer_is_stopped = true;

    xTimerStop(_control_loop_timer_handle, 0);
}

void ControllerMaster::_control_loop(TimerHandle_t timer)
{   
    /*if xTimerStop is not successfull, repeat xTimerStop*/
    if(_controller_obj->_timer_is_stopped != true)
    {   
        if(xSemaphoreTake(_controller_obj->_pos_controller_mutx, 0) == pdPASS)
        {
            //Input
            ros_msgs::Pose2D actual_pose = _controller_obj->_sensor_pose.getPose();

            //process position controller
            ros_msgs::Twist2D output_vel = _controller_obj->_pos_controller->update(actual_pose);

            ESP_LOGI("master", "time: %lld", esp_timer_get_time());

            //Output
            _controller_obj->_output_velocity.setVelocity(output_vel);

            if(_controller_obj->_pos_controller->destination_reached() == true)
                _controller_obj->_destination_reached_callback();

            xSemaphoreGive(_controller_obj->_pos_controller_mutx);
        }
    }
    else
        xTimerStop(_controller_obj->_control_loop_timer_handle, 0);
}