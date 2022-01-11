#pragma once

#include <functional>

#include "RosMsgs.h"
#include "SensorPose.h"
#include "NodeHandle.h"

class SensorPoseSim : public SensorPose 
{
    public:
        static SensorPose& init(ros::NodeHandle& node_handle);
        ros_msgs::Pose2D getPose() override;

    private:
        SensorPoseSim(ros::NodeHandle& node_handle);
        SensorPoseSim(SensorPoseSim const&) = delete;
        ~SensorPoseSim() {}

        void _setPose(ros_msgs::RosMsg const& pose);

        static SensorPoseSim* _sensor_pose_sim;

        ros_msgs::Pose2D _current_pose;

};

