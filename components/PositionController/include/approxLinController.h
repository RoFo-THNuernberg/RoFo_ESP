#pragma once

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"

#include "math.h"


class approxLinController : public PositionController
{
    public:
        explicit approxLinController(float damping_coefficient, float natural_frequency, ros_msgs::Trajectory const& trajectory);

        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        float _damping_coefficient;
        float _natural_frequency;

        ros_msgs::Trajectory const _trajectory;
        uint32_t _trajectory_cntr = 0;

        bool _destination_reached = false;
};