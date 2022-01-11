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

ros_msgs::Pose2D SensorPoseSim::getPose()
{
    return _current_pose;
}

void SensorPoseSim::_setPose(ros_msgs::RosMsg const& pose)
{   
    _current_pose = (ros_msgs::Pose2DSim&)pose;

}

