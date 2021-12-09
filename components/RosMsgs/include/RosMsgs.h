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
        virtual MSG_ID getMsgType() const = 0;
        virtual void deserialize(SmartBufferPtr bfr) = 0; 
    };

    struct String : RosMsg
    {   
        public:
            String(std::string data) : size{data.size() + 1}, data{data} {}

            void serialize(uint8_t* buffer) const override 
            { 
                memcpy(buffer, data.c_str(), data.size());
            }
            size_t getSize() const override { return size; }

            size_t const size;;
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

            MSG_ID getMsgType() const override
            {
                return _msg_type;
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
            MSG_ID const _msg_type = GEOMETRY_MSGS_POSE_2D_ID;
            size_t const _msg_size = 12;
    };

}