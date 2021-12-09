#pragma once

#include <string>

#include "msg_id.h"
#include "SmartBufferPtr.h"

namespace ros_msgs
{
    struct RosMsg
    {   
        virtual ~RosMsg() {}
        virtual void serialize(uint8_t* buffer) const = 0;
        virtual size_t getSize() const = 0;
        virtual void deserialize(SmartBufferPtr bfr) = 0; 
    };

    struct String : RosMsg
    {   
        public:
            String(std::string data) : data{data} {}

            size_t getSize() const override 
            { 
                return data.size() + 1; 
            }
            
            void serialize(uint8_t* buffer) const override 
            { 
                memcpy(buffer, data.c_str(), data.size());
            }
            
            void deserialize(SmartBufferPtr bfr)
            {
                data << bfr;
            }

            std::string data;
    };

    struct Pose2D : RosMsg
    {   
        public:
            Pose2D(float x, float y, float theta) : x{x}, y{y}, theta{theta} {}
            Pose2D() : x{0}, y{0}, theta{0} {}

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

            void deserialize(SmartBufferPtr bfr) override
            {
                x << bfr;
                bfr += 4;

                y << bfr;
                bfr += 4;

                theta << bfr;
            }

            float x;
            float y;
            float theta;

        private:
            size_t const _msg_size = 12;
    };

    struct Twist2D : RosMsg
    {
        public:
            Twist2D(float x, float w) : x{x}, w{w} {}
            Twist2D() : x{0}, w{0} {}

            size_t getSize() const override 
            { 
                return _msg_size; 
            }

            void serialize(uint8_t* buffer) const override 
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = w;
            }

            void deserialize(SmartBufferPtr bfr) override
            {
                x << bfr;
                bfr += 4;

                w << bfr;
                bfr += 4;
            }       

            float x;
            float w;

        private:
            size_t const _msg_size = 8;
    };

    struct Point2D : RosMsg
    {   
        public:
            Point2D(float x, float y) : x{x}, y{y} {}
            Point2D() : x{0}, y{0} {}

            size_t getSize() const override 
            { 
                return _msg_size; 
            }

            void serialize(uint8_t* buffer) const override 
            { 
                float* buff = (float*)buffer;
                buff[0] = x;
                buff[1] = y;
            }

            void deserialize(SmartBufferPtr bfr) override
            {
                x << bfr;
                bfr += 4;

                y << bfr;
                bfr += 4;
            }

            float x;
            float y;

        private:
            size_t const _msg_size = 8;
    };

}