#pragma once

#include "RosMsgs.h"
#include "NodeHandle.h"

class State;

class StateMachine
{
    public:
        StateMachine();

        void setState(State*);

        void set_velocity(ros_msgs::RosMsg const&);
        void set_goal_point(ros_msgs::RosMsg const&);
        void stop(ros_msgs::RosMsg const&);

    private:
        State* _current_state;

};
