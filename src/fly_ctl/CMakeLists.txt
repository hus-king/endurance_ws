cmake_minimum_required(VERSION 3.0.2)
project(fly_ctl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fly_ctl
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mavros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(fly_ctl
  src/motion_control.cpp
)

target_link_libraries(fly_ctl
  ${catkin_LIBRARIES}
)
