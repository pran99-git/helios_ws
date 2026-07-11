cmake_minimum_required(VERSION 3.8)
project(mapping_localization_pkg)

find_package(ament_cmake REQUIRED)

# Each mapping/localization technique gets its own top-level folder
# (rtabmap/ now; add more DIRECTORY entries here as new ones are added).
install(
  DIRECTORY rtabmap
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
