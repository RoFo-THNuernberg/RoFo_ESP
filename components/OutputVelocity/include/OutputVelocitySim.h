#pragma once

#include "NodeHandle.h"
#include "Publisher.h"

#include "OutputVelocity.h"

class OutputVelocitySim : public OutputVelocity
{
    public:
        static OutputVelocity& init(ros::NodeHandle& node_handle);
        void setVelocity(ros_msgs::Twist2D const& velocity) override;

    private:
        OutputVelocitySim(ros::NodeHandle& node_handle);
        OutputVelocitySim(OutputVelocitySim const&) = delete;
        ~OutputVelocitySim() {}

        ros::Publisher<ros_msgs::Twist2D> _publisher;
};