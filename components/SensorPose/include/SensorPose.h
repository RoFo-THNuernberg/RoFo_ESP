#pragma once

#include "esp_err.h"

#include "RosMsgs.h"

class SensorPose {
    public:
        virtual esp_err_t init() = 0;
        virtual const ros_msgs::Pose2D& get_Pose() = 0;
        ~SensorPose();
};

