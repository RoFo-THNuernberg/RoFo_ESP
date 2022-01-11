#pragma once

#include "RosMsgs.h"

class SensorPose {
    public:
        static SensorPose& getGlobalSensor();
        static void setGlobalSensor(SensorPose* global_sensor);
        virtual ros_msgs::Pose2D getPose() = 0;

    protected:
        virtual ~SensorPose() {}
        static SensorPose* _global_sensor;
        
};

