cmake_minimum_required(VERSION 2.8.3)
project(sciurus17_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sciurus17_examples
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

install(PROGRAMS
  scripts/pick_and_place_left_arm_demo.py
  scripts/pick_and_place_right_arm_demo.py
  scripts/gripper_action_example.py
  scripts/hand_position_publisher_example.py
  scripts/neck_joint_trajectory_example.py
  DESTINATION ${CATKIN_PACKESTINATION}
)
