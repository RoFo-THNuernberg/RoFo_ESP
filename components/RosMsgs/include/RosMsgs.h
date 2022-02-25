#pragma once

#include <string>
#include <vector>
#include <cstring>
#include <array>

#include "math.h"

#include "msg_id.h"

#include "esp_log.h"

namespace ros_msgs_lw
{
    struct Pose2D;
    struct Twist2D;
    struct Point2D;
}

namespace ros_msgs
{
    struct String
    {   
        public:
            explicit String() {}
            explicit String(std::string data) : data{data} {}

            size_t getSize() const
            { 
                if(data.empty() == true)
                    return 0; 
                
                return sizeof(int32_t) + data.size() + 1;
            }

            void setSize(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "std_msgs/String";
            }
            
            void serialize(uint8_t* buffer) const
            { 
                if(data.empty() == false)
                {
                    ((int32_t*)buffer)[0] = data.size() + 1;
                    memcpy(buffer + sizeof(int32_t), data.c_str(), data.size());
                    buffer[sizeof(int32_t) + data.size()] = '\0';
                }
            }
            
            void deserialize(uint8_t* buffer)
            {
                data.assign((char*)buffer);
            }

            bool operator==(std::string string_2) const
            {
                return data == string_2;
            }

            explicit operator std::string() const
            {
                return data;
            }

            std::string data;

    };

    struct Pose2DSim
    {   
        public:
            explicit Pose2DSim(float x, float y, float theta) : x{x}, y{y}, theta{theta} 
            {
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }
            Pose2DSim() : x{0}, y{0}, theta{0} {}

            size_t getSize() const
            { 
                return _msg_size; 
            }
            
            void setSize(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "turtlesim/Pose";
            }

            void serialize(uint8_t* buffer) const 
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = theta;
            }

            void deserialize(uint8_t* buffer)
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
            static size_t const _msg_size;
    };

    struct Pose2D
    {   
        public:
            explicit Pose2D(double x, double y, double theta) : x{x}, y{y}, theta{theta} 
            {
                //Keep theta between -pi and pi
                theta = atan2(sin(theta), cos(theta));
            }
            explicit Pose2D(ros_msgs::Pose2DSim const& pose) : x{pose.y}, y{pose.y}, theta{pose.theta} {}
            explicit Pose2D(ros_msgs_lw::Pose2D const& pose);

            Pose2D() : x{0}, y{0}, theta{0} {}

            size_t getSize() const 
            { 
                return _msg_size; 
            }
            
            void setSize(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "geometry_msgs/Pose2D";
            }

            void serialize(uint8_t* buffer) const
            { 
                double* buff = (double*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = theta;
            }

            void deserialize(uint8_t* buffer)
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
            static size_t const _msg_size;
    };

    struct Twist2D
    {
        public:
            explicit Twist2D(double x, double w) : v{x}, w{w} {}
            explicit Twist2D(ros_msgs_lw::Twist2D const& velocity);
            Twist2D() : v{0}, w{0} {}

            size_t getSize() const
            { 
                return _msg_size; 
            }
            
            void setSize(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "geometry_msgs/Twist";
            }

            void serialize(uint8_t* buffer) const
            { 
                double* buff = (double*)buffer;
                buff[0] = v;
                buff[1] = w;
            }

            void deserialize(uint8_t* buffer)
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

            void operator=(double i)
            {  
                v = i;
                w = i;
            }

            double v;
            double w;

        private:
            static size_t const _msg_size;
    };

    struct Point2D
    {   
        public:
            explicit Point2D(double x, double y) : x{x}, y{y} {}
            explicit Point2D(ros_msgs_lw::Point2D const& point);
            Point2D() : x{0}, y{0} {}

            size_t getSize() const 
            { 
                return _msg_size; 
            }
            
            void setSize(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "geometry_msgs/Point";
            }

            void serialize(uint8_t* buffer) const
            { 
                double* buff = (double*)buffer;
                buff[0] = x;
                buff[1] = y;
            }

            void deserialize(uint8_t* buffer)
            {
                double* buff = (double*)buffer;

                x = buff[0];
                y = buff[1];
            }

            double x;
            double y;

        private:
            static size_t const _msg_size;
    };

    struct TrajectoryStateVector
    {
        public:
            explicit TrajectoryStateVector(float x, float y, float dx, float dy, float ddx, float ddy, float theta, uint64_t timestamp) : x{x}, y{y}, dx{dx}, 
                dy{dy}, ddx{ddx}, ddy{ddy}, timestamp{timestamp} {}
            TrajectoryStateVector() : x{0}, y{0}, dx{0}, dy{0}, ddx{0}, ddy{0}, timestamp{0} {} 

            static size_t getSize()
            { 
                return _msg_size; 
            }

            void setSize(int32_t msg_len) {}

            static std::string getMsgType()
            {
                return "trajecgenerator/c_trajec";
            }

            void serialize(uint8_t* buffer) const
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = y;
                buff[2] = dx;
                buff[3] = dy;
                buff[4] = ddx;
                buff[5] = ddy;

                *reinterpret_cast<uint64_t*>(&buff[6]) = timestamp;
            }

            void deserialize(uint8_t* buffer)
            {   
                float* buff = (float*)buffer;

                x = buff[0];
                y = buff[1];
                dx = buff[2];
                dy = buff[3];
                ddx = buff[4];
                ddy = buff[5];

                timestamp = *reinterpret_cast<uint64_t*>(&buff[6]);
            }

            float x;
            float y;
            float dx;
            float dy;
            float ddx;
            float ddy;
            uint64_t timestamp;

        private:
            static size_t const _msg_size;
    };

    struct Trajectory
    {
        public:
            Trajectory() {}

            size_t getSize() const 
            {
                if(trajectory.empty() == true)
                    return 0;

                return trajectory.size() * TrajectoryStateVector::getSize() + sizeof(int32_t);
            }
            
            void setSize(int32_t msg_len) 
            {
                _trajectory_points = msg_len / TrajectoryStateVector::getSize();
            }

            static std::string getMsgType()
            {
                return "trajecgenerator/c_trajec_vector";
            }

            size_t getTrajectorySize() const
            {
                return trajectory.size();
            }

            void serialize(uint8_t* buffer) const
            {   
                if(trajectory.empty() == false)
                {
                    *reinterpret_cast<int32_t*>(buffer) = trajectory.size() * TrajectoryStateVector::getSize();
                    buffer += sizeof(int32_t);

                    for(auto i : trajectory)
                    {
                        i.serialize(buffer);

                        buffer += TrajectoryStateVector::getSize();
                    }
                }
            }

            void deserialize(uint8_t* buffer)
            {
                trajectory.clear();

                trajectory.reserve(_trajectory_points);

                for(int i = 0; i < _trajectory_points; i++)
                {   
                    TrajectoryStateVector trajectory_state;
                    trajectory_state.deserialize(buffer);

                    trajectory.push_back(trajectory_state);

                    buffer += TrajectoryStateVector::getSize();
                }
            }

            ros_msgs::TrajectoryStateVector operator[](int i) const
            {
                return trajectory[i];
            }

            std::vector<TrajectoryStateVector> trajectory;

        private:
            uint32_t _trajectory_points;
    };
}