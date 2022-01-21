#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "RosMsgsLw.h"

class SensorPose {
    public:
        bool getPose(ros_msgs_lw::Pose2D& current_pose) const;

        virtual void reInit() = 0;

    protected:
        SensorPose();
        ~SensorPose();

        QueueHandle_t _current_pose_queue;

    private:
        SensorPose(SensorPose const&) = delete;
        
};

