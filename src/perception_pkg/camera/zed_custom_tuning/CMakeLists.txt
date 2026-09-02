cmake_minimum_required(VERSION 3.8)
project(zed_custom_tuning)

find_package(ament_cmake REQUIRED)

# Files only -- no nodes. This package exists so our departures from
# Stereolabs' shipped ZED defaults live in the workspace rather than inside
# the zed-ros2-wrapper submodule, where `git submodule update` reverts them.
#
#   config/zed_overrides.yaml   passed to the wrapper's launch file as
#                               ros_params_override_path; see the README for
#                               where it lands in the precedence order.
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
