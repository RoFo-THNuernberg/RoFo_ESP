idf_component_register(SRCS "Marvelmind.cpp" "SensorPose.cpp" "SensorPoseSim.cpp" "KalmanFilter.cpp" "matrix_math.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES RosMsgs RosBridgeClient OutputVelocity)
