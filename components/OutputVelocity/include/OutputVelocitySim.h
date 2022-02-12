#pragma once

#include "NodeHandle.h"
#include "Publisher.h"
#include "OutputVelocity.h"
#include "RosMsgs.h"
#include "RosMsgsLw.h"

/**
 * @brief This class sends the robot velocity vector to the ROS Turtlesim simulator
 */
class OutputVelocitySim : public OutputVelocity
{
    public:
        /**
         * @brief Initialize the OutputVelocitySim instance
         * 
         * @note It is safe to call this function multiple times. It will only create one instance.
         * 
         * @return Reference to the OutputVelocity instance
         */  
        static OutputVelocity& init(ros::NodeHandle& node_handle);
        
        /**
         * @brief This function publishes the velocity vector to ROS. This can be useful for simulating the robot with e.g Turtlesim
         */ 
        void setVelocity(ros_msgs_lw::Twist2D const& velocity) override;

    private:
        OutputVelocitySim(ros::NodeHandle& node_handle);
        OutputVelocitySim(OutputVelocitySim const&) = delete;
        ~OutputVelocitySim() {}

        ros::Publisher<ros_msgs::Twist2D> _publisher;
};