#pragma once

#include "RosMsgs.h"

class SensorPose {
    public:
        virtual ~SensorPose() {}

        static SensorPose& getSensor();

        virtual ros_msgs::Pose2D get_Pose() = 0;

    protected:
        static SensorPose* _sensor;
};

