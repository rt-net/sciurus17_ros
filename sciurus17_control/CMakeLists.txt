cmake_minimum_required(VERSION 2.8.3)
project(sciurus17_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  dynamixel_sdk
  roscpp
  rospy
  hardware_interface
  transmission_interface
  sciurus17_msgs
  dynamic_reconfigure
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface transmission_interface controller_manager dynamixel_sdk
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR})

find_library( dxl_x64_cpp NAME libdxl_x64_cpp.so PATHS src/dynamixel_sdk/build/linux64 )

add_executable(sciurus17_control src/hardware.cpp src/joint_control.cpp src/dxlport_control.cpp src/control_setting.cpp)

target_link_libraries( sciurus17_control
  ${catkin_LIBRARIES}
)
set(MESSAGE_DEPENDS sciurus17_msgs_generate_messages)

install(TARGETS sciurus17_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
foreach(dir config launch scripts)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)