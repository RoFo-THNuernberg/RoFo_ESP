#pragma once

#include <functional>

#include "RosMsgs.h"
#include "SensorPose.h"
#include "NodeHandle.h"

class SensorPoseSim : public SensorPose 
{
    public:
        static SensorPose& init(ros::NodeHandle& node_handle);
        ros_msgs::Pose2D get_Pose() override;

    private:
        SensorPoseSim(ros::NodeHandle& node_handle);
        SensorPoseSim(SensorPoseSim const&) = delete;
        ~SensorPoseSim() {}

        void setPose(ros_msgs::RosMsg const& pose);

        ros_msgs::Pose2D _current_pose;

};

