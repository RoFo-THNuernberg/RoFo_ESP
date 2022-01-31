#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

#include <cstdio>
#include <cstdarg>
#include <memory>
#include <string>
#include <queue>

#include "Publisher.h"
#include "RosMsgs.h"


class DataLogger
{
    public:
        static DataLogger& init(ros::Publisher<ros_msgs::String>& publisher);

        void startLogCallback(std::shared_ptr<ros_msgs::String> msg);
        void logData(const char* format, ...);

        static DataLogger* _data_logger;

    private:
        DataLogger(ros::Publisher<ros_msgs::String>& publisher);
        DataLogger(DataLogger const&) = delete;
        ~DataLogger();

        static void _data_logger_task(void* pvParameters);

        ros::Publisher<ros_msgs::String>& _publisher;

        int _data_logging_time_ms = 0;

        std::queue<char*> _log_buffer_queue;
        int _log_buffer_cntr = 0;

        SemaphoreHandle_t _data_logging_semphr;

        TaskHandle_t _data_logger_task_handle;
};

#ifdef DATA_LOGGING
    #define LOG_DATA(format, ...) {                                     \
        if(DataLogger::_data_logger != nullptr)                         \
            DataLogger::_data_logger->logData(format, ##__VA_ARGS__);}   
#else
    #define LOG_DATA(format, ...) {}
#endif

