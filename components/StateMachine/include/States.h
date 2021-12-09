#pragma once

#include "RosMsgs.h"

class StateMachine;

class State
{
    public:
        virtual ~State() {}
        virtual void set_velocity(StateMachine&, ros_msgs::Twist2D const&) = 0;
        virtual void set_goal_point(StateMachine&, ros_msgs::Point2D const&) = 0;
        virtual void stop(StateMachine&, ros_msgs::String const&) = 0;
};

class Idle : public State
{
    public:
        void set_velocity(StateMachine&, ros_msgs::Twist2D const&) override;
        void set_goal_point(StateMachine&, ros_msgs::Point2D const&) override;
        void stop(StateMachine&, ros_msgs::String const&) override {}
};

class DriveWithVelocity : public State
{
    public:
        void set_velocity(StateMachine&, ros_msgs::Twist2D const&) override;
        void set_goal_point(StateMachine&, ros_msgs::Point2D const&) override {}
        void stop(StateMachine&, ros_msgs::String const&) override;
};

class DriveToPoint : public State
{
    public:
        void set_velocity(StateMachine&, ros_msgs::Twist2D const&) override {}
        void set_goal_point(StateMachine&, ros_msgs::Point2D const&) override;
        void stop(StateMachine&, ros_msgs::String const&) override;
};