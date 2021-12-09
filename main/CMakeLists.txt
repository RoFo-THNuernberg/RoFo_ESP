idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES SensorPose Wifi RosBridgeClient RosMsgs Socket nvs_flash)
