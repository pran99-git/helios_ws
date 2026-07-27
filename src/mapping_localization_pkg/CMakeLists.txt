cmake_minimum_required(VERSION 3.8)
project(mapping_localization_pkg)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY rtabmap
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
