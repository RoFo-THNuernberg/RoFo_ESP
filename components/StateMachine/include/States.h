#pragma once

#include "ControllerMaster.h"
#include "p2pController.h"
#include "OutputVelocity.h"
#include "RosMsgsLw.h"

#include <functional>
#include <string>

class StateMachine;

class State
{
    public:
        virtual ~State() {}
        virtual std::string getState() const = 0; 
        virtual void set_velocity(StateMachine&, ros_msgs_lw::Twist2D const&) = 0;
        virtual void set_goal_point(StateMachine&, ros_msgs_lw::Point2D const&) = 0;
        virtual void stop(StateMachine&) = 0;
};

class Idle : public State
{
    public:
        std::string getState() const override { return _state; }
        void set_velocity(StateMachine&, ros_msgs_lw::Twist2D const&) override;
        void set_goal_point(StateMachine&, ros_msgs_lw::Point2D const&) override;
        void stop(StateMachine&) override {}

    private:
        static std::string const _state;
};

class DriveWithVelocity : public State
{
    public:
        std::string getState() const override { return _state; }
        void set_velocity(StateMachine&, ros_msgs_lw::Twist2D const&) override;
        void set_goal_point(StateMachine&, ros_msgs_lw::Point2D const&) override {}
        void stop(StateMachine&) override;
        
    private:
        static std::string const _state;
};

class DriveToPoint : public State
{
    public:
        std::string getState() const override { return _state; }
        void set_velocity(StateMachine&, ros_msgs_lw::Twist2D const&) override;
        void set_goal_point(StateMachine&, ros_msgs_lw::Point2D const&) override;
        void stop(StateMachine&) override;
        
    private:
        static std::string const _state;
};