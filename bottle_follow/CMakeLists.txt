cmake_minimum_required(VERSION 2.8.3)
project(dnn_rotate)

## Find catkin macros and libraries
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS dnn_detect
)

