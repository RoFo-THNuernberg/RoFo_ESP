#include "SensorPoseSim.h"


SensorPoseSim::SensorPoseSim(ros::NodeHandle& node_handle)
{
    node_handle.subscribe<ros_msgs::Pose2DSim>("pose", std::bind(&SensorPoseSim::setPose, this, std::placeholders::_1));
}

SensorPose& SensorPoseSim::init(ros::NodeHandle& node_handle)
{
    if(_sensor == nullptr)
        _sensor = new SensorPoseSim(node_handle);

    return *_sensor;
}

ros_msgs::Pose2D SensorPoseSim::get_Pose()
{
    return _current_pose;
}

void SensorPoseSim::setPose(ros_msgs::RosMsg const& pose)
{   
    _current_pose = (ros_msgs::Pose2DSim&)pose;
}

