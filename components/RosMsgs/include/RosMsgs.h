#pragma once

#include <string>
#include <cstring>
#include <array>

#include "esp_log.h"
#include "math.h"

#include "msg_id.h"

namespace ros_msgs
{
   struct RosMsg
    {   
        virtual ~RosMsg() {}
        virtual void serialize(uint8_t* buffer) const = 0;
        virtual size_t getSize() const = 0;
        virtual void deserialize(uint8_t* buffer) = 0; 
    };

    struct String : RosMsg
    {   
        public:
            String(std::string data) : data{data} {}

            size_t getSize() const override 
            { 
                if(data.empty() == true)
                    return 0; 
                
                return data.size();
            }
            
            void serialize(uint8_t* buffer) const override 
            { 
                memcpy(buffer, data.c_str(), data.size());
            }
            
            void deserialize(uint8_t* buffer)
            {
                data.assign((char*)buffer);
            }

            std::string data;
    };

    struct Pose2DSim : RosMsg
    {   
        public:
            Pose2DSim(float x, float y, float theta) : x{x}, y{y}, theta{theta} 
            {
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }
            Pose2DSim() : x{0}, y{0}, theta{0} {}

            size_t getSize() const override 
            { 
                return _msg_size; 
            }

            void serialize(uint8_t* buffer) const override 
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = theta;
            }

            void deserialize(uint8_t* buffer) override
            {
                float* buff = (float*)buffer;

                x = buff[0];
                y = buff[1];
                theta = buff[2];

                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }

            float x;
            float y;
            float theta;

        private:
            size_t const _msg_size = 12;
    };

    struct Pose2D : RosMsg
    {   
        public:
            Pose2D(double x, double y, double theta) : x{x}, y{y}, theta{theta} 
            {
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }

            Pose2D() : x{0}, y{0}, theta{0} {}

            size_t getSize() const override 
            { 
                return _msg_size; 
            }

            void serialize(uint8_t* buffer) const override 
            { 
                double* buff = (double*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = theta;
            }

            void deserialize(uint8_t* buffer) override
            {
                double* buff = (double*)buffer;

                x = buff[0];
                y = buff[1];
                theta = buff[2];

                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }

            void operator=(Pose2D const& pose)
            {
                x = pose.x;
                y = pose.y;
                theta = pose.theta;

                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }

            void operator=(Pose2DSim const& pose)
            {
                x = pose.x;
                y = pose.y;
                theta = pose.theta;

                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }

            double x;
            double y;
            double theta;

        private:
            size_t const _msg_size = 24;
    };

    struct Twist2D : RosMsg
    {
        public:
            explicit Twist2D(double x, double w) : v{x}, w{w} {}
            Twist2D() : v{0}, w{0} {}

            size_t getSize() const override 
            { 
                return _msg_size; 
            }

            void serialize(uint8_t* buffer) const override 
            { 
                double* buff = (double*)buffer;
                buff[0] = v;
                buff[1] = w;
            }

            void deserialize(uint8_t* buffer) override
            {
                double* buff = (double*)buffer;

                v = buff[0];
                w = buff[1];
            }       

            void operator=(Twist2D const& velocity)
            {  
                v = velocity.v;
                w = velocity.w;
            }

            void operator=(int i)
            {  
                v = i;
                w = i;
            }

            double v;
            double w;

        private:
            size_t const _msg_size = 16;
    };

    struct Point2D : RosMsg
    {   
        public:
            Point2D(double x, double y) : x{x}, y{y} {}
            Point2D() : x{0}, y{0} {}

            size_t getSize() const override 
            { 
                return _msg_size; 
            }

            void serialize(uint8_t* buffer) const override 
            { 
                double* buff = (double*)buffer;
                buff[0] = x;
                buff[1] = y;
            }

            void deserialize(uint8_t* buffer) override
            {
                double* buff = (double*)buffer;

                x = buff[0];
                y = buff[1];
            }

            double x;
            double y;

        private:
            size_t const _msg_size = 16;
    };

    struct TrajectoryStateVector : RosMsg
    {
        public:
            explicit TrajectoryStateVector(float x, float y, float dx, float dy, float ddx, float ddy) : x{x}, y{y}, dx{dx}, 
                dy{dy}, ddx{ddx}, ddy{ddy} {}
            TrajectoryStateVector() : x{0}, y{0}, dx{0}, dy{0}, ddx{0}, ddy{0} {} 

            size_t getSize() const override 
            { 
                return _msg_size; 
            }

            void serialize(uint8_t* buffer) const override 
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = dx;
                buff[3] = dy;
                buff[4] = ddx;
                buff[5] = ddy;
            }

            void deserialize(uint8_t* buffer) override
            {   
                float* buff = (float*)buffer;

                x = buff[0];
                y = buff[1];
                dx = buff[2];
                dx = buff[3];
                ddx = buff[4];
                ddy = buff[5];
            }

            float x;
            float y;
            float dx;
            float dy;
            float ddx;
            float ddy;

        private:
            size_t const _msg_size = 24;
    };

    struct Trajectory : RosMsg
    {
        public:
            Trajectory() {}

            size_t getSize() const
            {
                return _trajectory_points * _trajectory->getSize();
            }

            size_t getTrajectorySize() const
            {
                return _trajectory_points;
            }

            void serialize(uint8_t* buffer) const override
            {   
                *((uint32_t*)buffer) = _trajectory_points;
                buffer += sizeof(_trajectory_points);

                for(int i = 0; i < _trajectory_points; i++)
                {
                    _trajectory[i].serialize(buffer);
                    buffer += _trajectory[i].getSize();
                }
            }

            void deserialize(uint8_t* buffer) override
            {
                if(_trajectory != nullptr)
                    delete[] _trajectory;
                
                _trajectory = new TrajectoryStateVector[_trajectory_points];

                for(int i = 0; i < _trajectory_points; i++)
                {
                    _trajectory[i].deserialize(buffer);
                    buffer += _trajectory[i].getSize();
                }
            }

            ros_msgs::TrajectoryStateVector& operator[](int i) const
            {
                return _trajectory[i];
            }

        private:
            uint32_t _trajectory_points;
            TrajectoryStateVector* _trajectory = nullptr;
    };

    Pose2D operator+(ros_msgs::Pose2D pose, std::array<float, 3> vector);
}