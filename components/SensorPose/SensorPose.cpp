#include "SensorPose.h"

SensorPose::SensorPose()
{
    _current_pose_queue = xQueueCreate(1, sizeof(ros_msgs_lw::Pose2D));
}

SensorPose::~SensorPose()
{
    vQueueDelete(_current_pose_queue);
}

bool SensorPose::getPose(ros_msgs_lw::Pose2D& current_pose) const
{
    if(xQueuePeek(_current_pose_queue, &current_pose, 0) == pdPASS)
        return true;

    return false;
}