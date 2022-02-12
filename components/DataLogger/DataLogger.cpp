#include "DataLogger.h"


#define DATA_LOG_SAMPLE_SIZE CONFIG_DATA_LOG_SAMPLE_SIZE
#define DATA_LOG_BUFFER_SIZE CONFIG_DATA_LOG_BUFFER_SIZE

#define MAX_LOG_TIME_SECONDS 60

#define TAG "DataLogger"

DataLogger* DataLogger::_data_logger = nullptr;


DataLogger::DataLogger(ros::Publisher<ros_msgs::String>& publisher) : _publisher{publisher} 
{
    _data_logger_task_handle = nullptr;

    _log_buffer_cntr = 0;

    _data_logging_semphr = xSemaphoreCreateBinary();
}

DataLogger::~DataLogger() 
{
    vSemaphoreDelete(_data_logging_semphr);
}

DataLogger& DataLogger::init(ros::Publisher<ros_msgs::String>& publisher)
{
    if(_data_logger == nullptr)
        _data_logger = new DataLogger(publisher);

    return *_data_logger;
}

void DataLogger::startLogCallback(std::shared_ptr<ros_msgs::String> msg)
{
    _data_logging_time_ms  = std::stof((std::string)*msg, nullptr) * 1000;

    if(_data_logging_time_ms > 0 && _data_logging_time_ms <= MAX_LOG_TIME_SECONDS * 1000)
    {
        if(_data_logger_task_handle == nullptr)
        {
            _log_buffer_queue.push(new char[DATA_LOG_BUFFER_SIZE]);
            _log_buffer_cntr = 0;

            xSemaphoreGive(_data_logging_semphr);
            xTaskCreate(_data_logger_task, "_data_logger_task", 2048, this, 3, &_data_logger_task_handle);
        }
    } 
}

void DataLogger::logData(const char* format, ...)
{                                    
    va_list arg;

    if(xSemaphoreTake(_data_logging_semphr, 0) == pdPASS)
    {
        if(_log_buffer_cntr + DATA_LOG_SAMPLE_SIZE > DATA_LOG_BUFFER_SIZE)
        {
            _log_buffer_queue.push(new char[DATA_LOG_BUFFER_SIZE]);  
            _log_buffer_cntr = 0;
        }

        va_start(arg, format);
        int len = vsnprintf(_log_buffer_queue.back() + _log_buffer_cntr, DATA_LOG_SAMPLE_SIZE, format, arg);  
        va_end(arg);

        if(len  != -1)
            _log_buffer_cntr += len;

        xSemaphoreGive(_data_logging_semphr);   
    }
}

void DataLogger::_data_logger_task(void *pvParameters)
{
    DataLogger& data_logger = *reinterpret_cast<DataLogger*>(pvParameters);

    vTaskDelay(data_logger._data_logging_time_ms / portTICK_PERIOD_MS);

    xSemaphoreTake(data_logger._data_logging_semphr, portMAX_DELAY);

    while(data_logger._log_buffer_queue.empty() == false)
    {
        ros_msgs::String msg;
        msg.deserialize(reinterpret_cast<uint8_t*>(data_logger._log_buffer_queue.front()));
        data_logger._publisher.publish(msg);

        delete[] data_logger._log_buffer_queue.front();

        data_logger._log_buffer_queue.pop();
    }

    data_logger._data_logger_task_handle = nullptr;
    vTaskDelete(NULL);
}