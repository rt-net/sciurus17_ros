cmake_minimum_required(VERSION 3.5)
project(sciurus17_examples)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set(LIBRARY_NAME "pose_presets")
add_library(${LIBRARY_NAME}
  SHARED
  src/${LIBRARY_NAME}.cpp
)
target_include_directories(
  ${LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  ${LIBRARY_NAME}
  angles
  geometry_msgs
  tf2_geometry_msgs
)

ament_export_targets(export_${LIBRARY_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  angles
  geometry_msgs
  tf2_geometry_msgs
)
install(
  DIRECTORY include/
  DESTINATION include/${LIBRARY_NAME}
)

install(
  TARGETS ${LIBRARY_NAME}
  EXPORT export_${LIBRARY_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Build and install node executables
set(executable_list
  gripper_control
  neck_control
  waist_control
  pick_and_place_right_arm_waist
  pick_and_place_left_arm
  head_camera_tracking
)
foreach(loop_var IN LISTS executable_list)
  add_executable(${loop_var} src/${loop_var}.cpp)
  target_link_libraries(${loop_var} ${LIBRARY_NAME})
  ament_target_dependencies(${loop_var}
    rclcpp_action
    cv_bridge
    geometry_msgs
    image_geometry
    moveit_ros_planning_interface
    OpenCV
    pcl_ros
    pcl_conversions
    rclcpp
    tf2_geometry_msgs
    control_msgs
  )

  install(TARGETS
    ${loop_var}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
