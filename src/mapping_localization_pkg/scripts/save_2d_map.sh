#!/usr/bin/env bash
# Save the current /rtabmap/map (nav_msgs/OccupancyGrid) as a standalone
# .pgm + .yaml pair, matched by name to a rtabmap.launch.py run's database
# (maps/rtabmap_<run_name>.db).
#
# Must be run WHILE rtabmap.launch.py is still up -- it saves a live topic,
# not something read from the .db file afterward.
#
# Run directly from source (not installed/ros2-run-wrapped -- it locates
# maps/ relative to its own location, which only resolves correctly here):
#   ~/helios_ws/src/mapping_localization_pkg/scripts/save_2d_map.sh <run_name>
#   ~/helios_ws/src/mapping_localization_pkg/scripts/save_2d_map.sh 20260709_143000
#
# Output: maps/rtabmap_<run_name>.pgm, maps/rtabmap_<run_name>.yaml

set -euo pipefail

if [ $# -ne 1 ]; then
    echo "Usage: $0 <run_name>" >&2
    echo "  <run_name> should match the 'run_name' the mapping run was launched with" >&2
    exit 1
fi

RUN_NAME="$1"
MAPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../maps" && pwd)"
OUT_PREFIX="${MAPS_DIR}/rtabmap_${RUN_NAME}"

echo "Saving /rtabmap/map -> ${OUT_PREFIX}.pgm / ${OUT_PREFIX}.yaml"
ros2 run nav2_map_server map_saver_cli -f "${OUT_PREFIX}" --occ /rtabmap/map
