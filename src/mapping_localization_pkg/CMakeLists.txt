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
  DIRECTORY
    rtabmap/launch
    rtabmap/rviz
  DESTINATION share/${PROJECT_NAME}/rtabmap
)

install(
  DIRECTORY
    slam_toolbox/launch
    slam_toolbox/config
    slam_toolbox/rviz
  DESTINATION share/${PROJECT_NAME}/slam_toolbox
)

# localization/ reuses a map instead of building one (AMCL over a saved
# .pgm/.yaml). Same layout rule as above: launch/ and config/ install,
# maps/ does not. scripts/ is deliberately NOT installed -- save_slam.sh
# locates maps/ relative to its own path and is meant to be run straight out
# of the source tree.
install(
  DIRECTORY
    localization/launch
    localization/config
  DESTINATION share/${PROJECT_NAME}/localization
)

# The only Python in this package: amcl_recovery re-localizes the rover without
# a human, so it has to be a running node rather than config.
#
# Both files go to lib/<pkg>/ rather than a Python package in site-packages.
# Python puts a script's own directory first on sys.path, so the node's
# `from amcl_recovery import ...` resolves to its sibling with no package
# machinery, and the two files stay next to each other in the source tree.
install(
  PROGRAMS
    localization/amcl_recovery_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME amcl_recovery_node
)

install(
  FILES
    localization/amcl_recovery.py
  DESTINATION lib/${PROJECT_NAME}
)

# exploration/ turns the growing SLAM map into goals, so it is mapping's
# consumer rather than a fourth mapping approach. Same layout rule as above:
# launch/ and config/ install, test/ does not.
install(
  DIRECTORY
    exploration/launch
    exploration/config
  DESTINATION share/${PROJECT_NAME}/exploration
)

# Same sibling-import pattern as localization/ above: the nodes go to
# lib/<pkg>/ beside the domain modules they import, so `from
# frontier_detection import ...` resolves with no package machinery.
# One call each: CMake's RENAME takes a single file, and the extension is
# dropped so `ros2 run`/`executable=` names match the other nodes here.
install(
  PROGRAMS exploration/frontier_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME frontier_node
)

install(
  PROGRAMS exploration/goal_selector_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME goal_selector_node
)

install(
  FILES
    exploration/frontier_detection.py
    exploration/goal_selection.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # The domain modules are pure Python and import no ROS, so they test
  # without a running graph. PYTHONPATH points at the source folder because
  # the modules are installed flat into lib/, not as a Python package.
  ament_add_pytest_test(exploration_tests exploration/test
    ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/exploration
  )
endif()

ament_package()
