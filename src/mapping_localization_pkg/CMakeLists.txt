cmake_minimum_required(VERSION 3.8)
project(mapping_localization_pkg)

find_package(ament_cmake REQUIRED)

# Each mapping approach lives in its own self-contained folder (launch/, plus
# config/ + rviz/ for slam_toolbox, plus its own maps/ for run outputs), so
# either can be run independently of the other:
#   ros2 launch mapping_localization_pkg slam_toolbox.launch.py
#   ros2 launch mapping_localization_pkg rtabmap.launch.py
# `ros2 launch` searches share/<pkg> recursively, so the nesting is transparent.
#
# Subdirectories are listed explicitly rather than installing whole folders
# with PATTERN ... EXCLUDE -- the exclude is silently bypassed under
# `colcon build --symlink-install`, which symlinks whole directories. This
# matters because maps/ must NOT be installed: RTAB-Map databases run to tens
# of GB, and a plain (non-symlink) build would copy all of them into install/
# on every build. Both launch files read/write maps/ in the source tree.
install(
  DIRECTORY rtabmap/launch
  DESTINATION share/${PROJECT_NAME}/rtabmap
)

install(
  DIRECTORY
    slam_toolbox/launch
    slam_toolbox/config
    slam_toolbox/rviz
  DESTINATION share/${PROJECT_NAME}/slam_toolbox
)

ament_package()
