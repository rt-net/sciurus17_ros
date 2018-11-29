cmake_minimum_required(VERSION 2.8.3)
project(sciurus17_tools)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

catkin_package(
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${realsense_INCLUDE_DIR}
)

add_executable(realsense_hwreset src/realsense_hwreset.cpp)
target_link_libraries(realsense_hwreset
    ${realsense2_LIBRARY}
    ${catkin_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

install(DIRECTORY scripts rules DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

