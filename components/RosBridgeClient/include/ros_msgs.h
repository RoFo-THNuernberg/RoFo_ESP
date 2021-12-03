#pragma once

#include <string>

namespace ros_msgs
{
    struct String 
    {   
        public:
            String(std::string data) : size{data.size() + 1}, data{data} {}

            uint8_t *getBuffer() const { return (uint8_t*)data.c_str(); }
            size_t getSize() const { return size; }

            size_t const size;;
            std::string data;
    };



    struct __attribute__((packed, aligned(1))) Pose2D
    {   
        Pose2D(float x, float y, float theta) : x{x}, y{y}, theta{theta} {}

        uint8_t *getBuffer() const { return (uint8_t*)this; }
        size_t getSize() const { return sizeof(Pose2D); }

        static std::string msg_type;
        float x;
        float y;
        float theta;
    };

}