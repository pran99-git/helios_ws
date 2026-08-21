cmake_minimum_required(VERSION 3.8)
project(sensor_fusion)

find_package(ament_cmake REQUIRED)

# What stays here is the FUSION layer's own content: ekf.yaml, the EKF launch,
# the whole-stack bring-up, and the RViz layout that shows camera and laser
# together. Per-sensor config and launch live beside their own vendor
# submodules instead -- Camera/custom_covariance and LiDAR/custom_config -- so
# that everything for a given sensor is in one place. The single-sensor laser
# layout moved with it (LiDAR/custom_config/rviz/lidar_scan.rviz).
#
# Nothing can sit directly in perception_pkg/: it is a plain container
# directory with no package.xml, so anything placed at that level is
# uninstallable and reachable only by absolute source path.
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
