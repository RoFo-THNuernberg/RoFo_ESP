idf_component_register(SRCS "NodeHandle.cpp" "Publisher.cpp" "ros_msgs.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES Socket data_types)
