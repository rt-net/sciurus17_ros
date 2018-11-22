cmake_minimum_required(VERSION 2.8.3)
project(sciurus17_vision)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_geometry
  image_transport
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_ros
  sciurus17_msgs
  dynamic_reconfigure
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(MESSAGE_DEPENDS sciurus17_msgs_generate_messages)
