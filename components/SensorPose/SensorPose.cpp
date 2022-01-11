#include "SensorPose.h"

SensorPose* SensorPose::_global_sensor = nullptr;

void SensorPose::setGlobalSensor(SensorPose* global_sensor)
{
    _global_sensor = global_sensor;
}

SensorPose& SensorPose::getGlobalSensor()
{
    return *_global_sensor;
}