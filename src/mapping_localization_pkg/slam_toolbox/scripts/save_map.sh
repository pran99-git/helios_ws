#!/usr/bin/env bash
# Save slam_toolbox's current /map (nav_msgs/OccupancyGrid) as a standalone
# .pgm + .yaml pair into slam_toolbox/maps/.
#
# Must be run WHILE slam_toolbox.launch.py is still up -- it saves a live topic,
# not something read back from a file afterward.
#
# Run directly from source (not installed/ros2-run-wrapped -- it locates maps/
# relative to its own location, which only resolves correctly here):
#   ~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts/save_map.sh
#   ~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts/save_map.sh lab_corridor
#
# With no argument the map is named by timestamp, matching how
# rtabmap.launch.py names its runs.
#
# Output: slam_toolbox/maps/slam_toolbox_<name>.pgm + .yaml

set -euo pipefail

if [ $# -gt 1 ]; then
    echo "Usage: $0 [name]" >&2
    echo "  [name]  optional label for this map; defaults to a timestamp" >&2
    exit 1
fi

NAME="${1:-$(date +%Y%m%d_%H%M%S)}"
MAPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../maps" && pwd)"
OUT_PREFIX="${MAPS_DIR}/slam_toolbox_${NAME}"

if [ -e "${OUT_PREFIX}.pgm" ] || [ -e "${OUT_PREFIX}.yaml" ]; then
    echo "Refusing to overwrite existing map: ${OUT_PREFIX}.pgm/.yaml" >&2
    echo "Pick a different name, or move the old one aside first." >&2
    exit 1
fi

echo "Saving /map -> ${OUT_PREFIX}.pgm / ${OUT_PREFIX}.yaml"
ros2 run nav2_map_server map_saver_cli -f "${OUT_PREFIX}" --occ /map
