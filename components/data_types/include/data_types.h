#pragma once

namespace data_types 
{
    class Pose2D {
        public:
            Pose2D() : x{0}, y{0}, theta{0} {};
            Pose2D(float x, float y, float theta) : x{x}, y{y}, theta{theta} {};

            float x, y, theta;
    };
};
