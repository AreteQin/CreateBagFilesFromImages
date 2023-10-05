cmake_minimum_required(VERSION 3.0.2)

project(BagFromImages)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        rosbag
        cv_bridge
        image_transport
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

FIND_PACKAGE(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
        main.cc
        Thirdparty/DLib/FileFunctions.cpp)

target_link_libraries(${PROJECT_NAME}
        console_bridge
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES})
