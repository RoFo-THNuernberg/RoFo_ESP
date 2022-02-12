#pragma once

#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "OutputVelocity.h"
#include "SensorPose.h"

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_err.h"

/**
 * @brief This class manages the Position Controller. It provides a timer triggered task which updates the Position Controller, checks if the destination was reached and then calls the provided callback_function.
 */
class ControllerMaster
{
    public:

        /**
         * @brief Initialize the ControllerMaster instance
         * 
         * @note It is safe to call this function multiple times. It will only create one instance.
         * 
         * @return Reference to the ControllerMaster instance
         */  
        static ControllerMaster& init(OutputVelocity& output_velocity, SensorPose& sensor_pose);

        /**
         * @brief This function starts the navigation. The navigation type (drive to point or trajectory tracking) is dependent on the provided PositonController object.
         * If a previous navigation gets interrupted by a new call to start_controller(), the old PositionController object is deleted/replaced by the new PosController and the new navigation is started.  
         * 
         * @param pos_controller Pointer to a PositionController object. The referenced object must be allocated on the heap. Deletion gets also handled by the ControllerMaster.
         * @param destination_reached_callback Callback function gets called when the PositionController reaches its goal. 
         */ 
        void start_controller(PositionController* pos_controller, std::function<void()>destination_reached_callback);
        
        /**
         * @brief This function stops the navigation. Since the ControllerMaster wont stop the PositionController automatically if the destination has been reached, it makes sense to call stop_controller() in the _destination_reached_callback function
         */ 
        void stop_controller();


    private:
        ControllerMaster(OutputVelocity& ouput_velocity, SensorPose& sensor_pose);
        ControllerMaster(const ControllerMaster&) = delete;
        ~ControllerMaster();

        static void _control_loop_timer(TimerHandle_t timer);
        static void _control_loop_task(void* pvParameters);

        static ControllerMaster* _controller_obj;

        uint64_t prev_time = 0;

        PositionController* _pos_controller;
        SemaphoreHandle_t _pos_controller_mutx;

        std::function<void()>_destination_reached_callback;

        OutputVelocity& _output_velocity;
        SensorPose& _sensor_pose;

        TimerHandle_t _control_loop_timer_handle;
        TaskHandle_t _control_loop_task_handle;

        bool _controller_is_stopped;
};
