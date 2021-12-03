idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES data_types SensorPose Wifi RosBridgeClient Socket nvs_flash)
