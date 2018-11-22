cmake_minimum_required(VERSION 2.8.3)
project(sciurus17_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  message_generation
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  actionlib_msgs
#   actionlib_msgs#   geometry_msgs#   sensor_msgs#   std_msgs
)

generate_dynamic_reconfigure_options(
    cfg/ServoParameter.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

