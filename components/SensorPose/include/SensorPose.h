#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "RosMsgsLw.h"

class SensorPose {
    public:
        virtual bool peekAtPose(ros_msgs_lw::Pose2D& current_pose) const = 0;
        virtual bool getPose(ros_msgs_lw::Pose2D& current_pose) const = 0;
        virtual void reInit() = 0;

    protected:
        SensorPose() {}
        ~SensorPose() {}
        SensorPose(SensorPose const&) = delete;
        
};

