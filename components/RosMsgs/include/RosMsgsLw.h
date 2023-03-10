#pragma once

#include "esp_err.h"
#include "math.h"

#include "mat.h"

namespace ros_msgs
{
    struct Pose2D;
    struct Pose2DSim;
    struct Twist2D;
    struct Point2D;
}

/**
 * @brief Since double precision arithmetic is only emulated on the ESP32, 
 * the ros_msgs_lw classes can be used to maximize execution speed.
 * These classes should provide a type casting constructor for ros_msgs.
 */
namespace ros_msgs_lw
{
    struct Pose2D
    {   
        public:
            explicit Pose2D(float x, float y, float theta);
            explicit Pose2D(ros_msgs::Pose2D const& pose);
            explicit Pose2D(ros_msgs::Pose2DSim const& pose);
            explicit Pose2D(dspm::Mat const& pose);

            Pose2D() : x{0}, y{0}, theta{0} {}

            void operator=(Pose2D const& pose);

            void operator=(dspm::Mat const& pose);

            float x;
            float y;
            float theta;
    };

    struct Twist2D
    {
        public:
            explicit Twist2D(float x, float w) : v{x}, w{w} {}
            explicit Twist2D(ros_msgs::Twist2D const& velocity);
            Twist2D() : v{0}, w{0} {}      

            void operator=(Twist2D const& velocity);

            void operator=(ros_msgs::Twist2D const& velocity);

            void operator=(float i);

            float v;
            float w;
    };

    struct Point2D
    {   
        public:
            explicit Point2D(float x, float y) : x{x}, y{y} {}
            explicit Point2D(ros_msgs::Point2D const& point);
            Point2D() : x{0}, y{0} {}

            float x;
            float y;

    };

    Pose2D operator*(float scalar, Pose2D const& pose);

    Pose2D operator+(Pose2D const& pose_1, Pose2D const& pose_2);

    Pose2D operator-(Pose2D const& pose_1, Pose2D const& pose_2);

    Pose2D operator+(Pose2D const& pose, dspm::Mat const& mat);

    dspm::Mat operator*(dspm::Mat const& mat, Pose2D const& pose);

}