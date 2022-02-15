#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_timer.h"

#include <cstdio>
#include <cstdarg>
#include <memory>
#include <string>
#include <queue>

#include "Publisher.h"
#include "RosMsgs.h"

/**
 * @brief Class for logging string data to ROS.
 * The logs are accumulated to large String blocks and then published to ROS.
 * 
 * @note check the Data Logger section in idf.py menuconfig to configure the behaviour
 */
class DataLogger
{
    public:
        /**
         * @brief Method for initializing the DataLogger.
         * 
         * @param [in] publisher: Publisher for sending LogData to ROS
         * @return DataLogger object
         */
        static DataLogger& init(ros::Publisher<ros_msgs::String>& publisher);

        /**
         * @brief Start logging for specified time
         * 
         * @note This method is intended as a callback function for the RosBridgeClient Subscriber
         * 
         * @param [in] log_time floating point time in seconds
         */
        void startLogging(std::shared_ptr<ros_msgs::String> log_time);

        /**
         * @brief Method for logging user formated string data
         * 
         * @note It is recommended to use the macro LOG_DATA()
         * 
         * @param [in] format C format string
         * @param [in] ... additional arguments depending on the format string
         */
        void logData(const char* format, ...);

        static DataLogger* _data_logger;

    private:
        DataLogger(ros::Publisher<ros_msgs::String>& publisher);
        DataLogger(DataLogger const&) = delete;
        ~DataLogger();
        
        ///task for publishing data
        static void _data_logger_task(void* pvParameters);

        ros::Publisher<ros_msgs::String>& _publisher;

        ///log until the specified time
        uint64_t _data_logging_end_us = 0;

        ///accumulate data before adding it to the queue
        char* _log_buffer;
        int _log_buffer_cntr = 0;

        ///queue contains char pointers to the data blocks
        QueueHandle_t _log_buffer_queue;

        ///semaphore for controlling when to add new data in the logData() method
        SemaphoreHandle_t _data_logging_semphr;

        TaskHandle_t _data_logger_task_handle;
};


/**
 * \class DataLogger
 * \def LOG_DATA(format, ...)
 * 
 * @brief Macro for logging data 
 * 
 * @note DATA_LOGGING must be defined before including DataLogger.h to enable logging
 * 
 * @param [in] format C format string
 * @param [in] ... additional arguments depending on the format string
*/
#ifdef DATA_LOGGING
    #define LOG_DATA(format, ...) {                                     \
        if(DataLogger::_data_logger != nullptr)                         \
            DataLogger::_data_logger->logData(format, ##__VA_ARGS__);}   
#else
    #define LOG_DATA(format, ...) {}
#endif

