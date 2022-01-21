idf_component_register(SRCS "Marvelmind.cpp" "SensorPose.cpp" "SensorPoseSim.cpp" "KalmanFilter.cpp" "KalmanSensor.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES RosBridgeClient OutputVelocity esp-dsp)
