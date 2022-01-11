#pragma once

#include <array>    

#include "RosMsgs.h"

class KalmanSensor
{
    public:
        virtual bool newData() = 0;
        //virtual void calculateKalmanGain(std::array<std::array<float, 3>, 3> const& a_priori_cov) = 0;
        //virtual ros_msgs::Pose2D getPosterioriPose(ros_msgs::Pose2D const& a_priori_pose) = 0;
        //virtual std::array<std::array<float, 3>, 3> getPosterioriCov(std::array<std::array<float, 3>, 3> const&) = 0;
};