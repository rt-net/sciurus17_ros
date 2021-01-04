cmake_minimum_required(VERSION 2.8.3)
project(sciurus17_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_msgs
  std_srvs
)

# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

###########
## Build ##
###########
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(GLOB python_scripts scripts/*.py)
catkin_install_python(
  PROGRAMS ${python_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
