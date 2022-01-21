#pragma once

#include <functional>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "RosMsgsLw.h"
#include "RosMsgs.h"
#include "SensorPose.h"
#include "NodeHandle.h"

class SensorPoseSim : public SensorPose 
{
    public:
        static SensorPose& init(ros::NodeHandle& node_handle);

        void reInit() override {}

    private:
        SensorPoseSim(ros::NodeHandle& node_handle);
        SensorPoseSim(SensorPoseSim const&) = delete;
        ~SensorPoseSim() {}

        void _setPose(ros_msgs::RosMsg const& pose_msg);

        static SensorPoseSim* _sensor_pose_sim;

};

