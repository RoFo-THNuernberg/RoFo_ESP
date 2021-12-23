#include "SensorPose.h"

SensorPose* SensorPose::_sensor = nullptr;

SensorPose& SensorPose::getSensor()
{
    return *_sensor;
}