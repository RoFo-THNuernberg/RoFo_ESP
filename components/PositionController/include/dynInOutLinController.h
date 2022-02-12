#pragma once

#include "PositionController.h"
#include "RosMsgsLw.h"
#include "RosMsgs.h"

#include "math.h"
#include "esp_timer.h"


/**
 * @brief PositionController implementation -> check Projektarbeit Regelung und Simulation von Schwarmrobotern
 */
class dynInOutLinController : public PositionController
{
    public:
        explicit dynInOutLinController(float kp_1, float kp_2, float kd_1, float kd_2, ros_msgs::Trajectory const& trajectory);
        ros_msgs_lw::Twist2D update(ros_msgs_lw::Pose2D const& actual_pose) override;
        bool destination_reached() override;

    private:
        ros_msgs_lw::Pose2D _prev_pose;  //TODO: must be initialized with first value of trajectory
        
        float _kp_1;
        float _kp_2;
        float _kd_1;
        float _kd_2;

        uint64_t _prev_time_us;
        float _prev_velocity;
        float _prev_acceleration;

        ros_msgs::Trajectory const _trajectory;
        uint32_t _trajectory_cntr = 0;

        bool _destination_reached = false;
};