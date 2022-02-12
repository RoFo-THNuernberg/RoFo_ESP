#pragma once

#include "math.h"

#include "PositionController.h"
#include "RosMsgsLw.h"

#include "esp_log.h"


/**
 * @brief PositionController implementation -> check Projektarbeit Regelung und Simulation von Schwarmrobotern
 */
class p2pController : public PositionController
{
    public:
        explicit p2pController(float kp_v, float kp_w, ros_msgs_lw::Point2D const& goal_point);

        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        float _kp_v;
        float _kp_w;

        ros_msgs_lw::Point2D const _goal_point;
        
        bool _destination_reached = false;
};