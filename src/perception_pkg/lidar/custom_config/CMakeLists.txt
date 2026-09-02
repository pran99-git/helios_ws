cmake_minimum_required(VERSION 3.8)
project(custom_config)

find_package(ament_cmake REQUIRED)

# Files only -- no nodes. This package exists so the Hokuyo's parameters and
# launch live in the workspace rather than inside the urg_node2 submodule,
# where `git submodule update` would revert them.
#
#   config/urg_node2.yaml   driver parameters (IP, angle limits, frame_id)
#   launch/lidar.launch.py  replaces the submodule's launch, which hardcodes
#                           its own parameter file path with no override
#   rviz/lidar_scan.rviz    layout for checking the laser on its own
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
