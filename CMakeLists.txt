cmake_minimum_required(VERSION 2.8.3)
project(gps_mux_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES record_service
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
