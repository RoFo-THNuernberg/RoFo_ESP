#pragma once

#include <memory>

#include "RosMsgs.h"
#include "RosMsgsLw.h"
#include "OutputVelocity.h"
#include "ControllerMaster.h"

class State;

class StateMachine
{
    public:
        static StateMachine& init(ControllerMaster& controller_master, OutputVelocity& output_velocity);

        void setState(State*);

        void set_velocity(std::shared_ptr<ros_msgs::Twist2D> vel_vector);
        void set_goal_point(std::shared_ptr<ros_msgs::Point2D> goal_point);
        void set_trajectory(std::shared_ptr<ros_msgs::Trajectory> trajectory);
        void stop();

        ControllerMaster& controller_master;
        OutputVelocity& output_velocity;

    private:
        StateMachine(ControllerMaster& controller_master, OutputVelocity& output_velocity);
        StateMachine(StateMachine const&) = delete;
        ~StateMachine() {}

        static StateMachine* _state_machine_obj;

        State* _current_state;

};
