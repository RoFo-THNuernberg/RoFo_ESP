#pragma once

namespace data_types 
{
    class Pose {
        public:
            Pose() : x{0}, y{0}, theta{0} {};
            Pose(float new_x, float new_y, float new_theta) : x{new_x}, y{new_y}, theta{new_theta} {};

            float x, y, theta;
    };
};
