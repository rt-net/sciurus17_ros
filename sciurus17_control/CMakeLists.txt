cmake_minimum_required(VERSION 3.5)
project(sciurus17_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
