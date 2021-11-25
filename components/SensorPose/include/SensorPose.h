#pragma once

#include "esp_err.h"

#include "data_types.h"

class SensorPose {
    public:
        virtual esp_err_t init() = 0;
        virtual const data_types::Pose& get_Pose() = 0;
        ~SensorPose();
};

