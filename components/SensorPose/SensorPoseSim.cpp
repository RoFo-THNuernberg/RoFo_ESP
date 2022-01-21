#include "SensorPoseSim.h"

SensorPoseSim* SensorPoseSim::_sensor_pose_sim = nullptr;

SensorPoseSim::SensorPoseSim(ros::NodeHandle& node_handle)
{
    node_handle.subscribe<ros_msgs::Pose2DSim>("pose", std::bind(&SensorPoseSim::_setPose, this, std::placeholders::_1));
}

SensorPose& SensorPoseSim::init(ros::NodeHandle& node_handle)
{
    if(_sensor_pose_sim == nullptr)
        _sensor_pose_sim = new SensorPoseSim(node_handle);

    return *_sensor_pose_sim;
}

void SensorPoseSim::_setPose(ros_msgs::RosMsg const& pose_msg)
{   
    ros_msgs_lw::Pose2D current_pose((ros_msgs::Pose2DSim&)pose_msg);

    xQueueOverwrite(_current_pose_queue, &current_pose);
}

