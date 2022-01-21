#pragma once

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

        void set_velocity(ros_msgs::RosMsg const&);
        void set_goal_point(ros_msgs::RosMsg const&);
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
