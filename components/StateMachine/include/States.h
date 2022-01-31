#pragma once

#include "ControllerMaster.h"
#include "p2pController.h"
#include "OutputVelocity.h"
#include "RosMsgsLw.h"

#include <functional>
#include <memory>
#include <string>

class StateMachine;

class State
{
    public:
        virtual ~State() {}
        virtual std::string getState() const = 0; 
        virtual void set_velocity(StateMachine&, std::shared_ptr<ros_msgs::Twist2D>) = 0;
        virtual void set_goal_point(StateMachine&, std::shared_ptr<ros_msgs::Point2D>) = 0;
        virtual void stop(StateMachine&) = 0;
};

class Idle : public State
{
    public:
        std::string getState() const override { return _state; }
        void set_velocity(StateMachine&, std::shared_ptr<ros_msgs::Twist2D>) override;
        void set_goal_point(StateMachine&, std::shared_ptr<ros_msgs::Point2D>) override;
        void stop(StateMachine&) override {}

    private:
        static std::string const _state;
};

class DriveWithVelocity : public State
{
    public:
        std::string getState() const override { return _state; }
        void set_velocity(StateMachine&, std::shared_ptr<ros_msgs::Twist2D>) override;
        void set_goal_point(StateMachine&, std::shared_ptr<ros_msgs::Point2D>) override {}
        void stop(StateMachine&) override;
        
    private:
        static std::string const _state;
};

class DriveToPoint : public State
{
    public:
        std::string getState() const override { return _state; }
        void set_velocity(StateMachine&, std::shared_ptr<ros_msgs::Twist2D>) override;
        void set_goal_point(StateMachine&, std::shared_ptr<ros_msgs::Point2D>) override;
        void stop(StateMachine&) override;
        
    private:
        static std::string const _state;
};