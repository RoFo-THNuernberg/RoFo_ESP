#pragma once

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"

#include "math.h"


class statInOutLinController : public PositionController
{
    public:
        explicit statInOutLinController(float kp_1, float kp_2, float b_offset, ros_msgs::Trajectory const& trajectory);
        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        float _kp_1;
        float _kp_2;
        float _b_offset;

        ros_msgs::Trajectory const _trajectory;
        uint32_t _trajectory_cntr = 0;

        bool _destination_reached = false;
};