cmake_minimum_required(VERSION 3.10)
project(edubot_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch maps param 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
