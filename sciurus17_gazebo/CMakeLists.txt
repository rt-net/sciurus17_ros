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

#############
## Install ##
#############
foreach(dir config launch models scripts worlds)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)
