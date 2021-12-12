#include "ControllerMaster.h"


ControllerMaster::ControllerMaster() 
{   
    /*maybe adjust freeRtos tick rate*/
    _control_loop_handle = xTimerCreate("control_loop", pdMS_TO_TICKS(10), pdTRUE, NULL, _control_loop);
}

ControllerMaster& ControllerMaster::init()
{
    return _controller_obj;
}

void ControllerMaster::_control_loop(TimerHandle_t timer)
{
    
}