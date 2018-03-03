cmake_minimum_required(VERSION 2.8.3)
project(move_demo)

## Find catkin macros and libraries
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS rospy move_base_msgs)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS move_basic
)
