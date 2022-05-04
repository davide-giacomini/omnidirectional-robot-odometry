cmake_minimum_required(VERSION 2.8.3)
project(omnidirectional-robot-odometry)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

## Declare a catkin package
catkin_package()

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(sub src/sub.cpp)
target_link_libraries(sub ${catkin_LIBRARIES})
