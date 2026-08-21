#!/usr/bin/env bash
# Save a slam_toolbox mapping run in BOTH of its output formats -- all four
# files -- under one shared name.
#
#   <prefix>.pgm + <prefix>.yaml        the rendered occupancy grid. An image
#                                       plus its metadata. This is what AMCL
#                                       (localization/amcl_localization.
#                                       launch.py) and nav2's costmaps consume.
#                                       Lossy: there is no pose graph behind
#                                       it, so slam_toolbox itself cannot
#                                       resume, extend, or localize from it.
#
#   <prefix>.posegraph + <prefix>.data  the full SLAM session: nodes, scans,
#                                       constraints, loop closures. The only
#                                       thing slam_toolbox's own localization
#                                       mode can load, and the only format you
#                                       can continue mapping from later.
#
# Neither pair can be regenerated from the other, which is why this saves both
# by default. Doing it in one call also keeps the two halves on the SAME name:
# running two separate scripts a minute apart left you with an occupancy grid
# and a pose graph timestamped differently, with nothing recording that they
# came from the same run.
#
# Must be run WHILE slam_toolbox.launch.py is still up -- both halves are live
# service/topic captures, not something read back off disk afterward.
#
# Run directly from source (not installed / ros2-run-wrapped -- it locates
# maps/ relative to its own location, which only resolves correctly here):
#
#   S=~/helios_ws/src/mapping_localization_pkg/slam_toolbox/scripts
#   $S/save_slam.sh                  # both formats, named by timestamp
#   $S/save_slam.sh lab_corridor     # both formats, named lab_corridor
#   $S/save_slam.sh lab_corridor --map-only     # just .pgm + .yaml
#   $S/save_slam.sh lab_corridor --graph-only   # just .posegraph + .data
#
# Output: slam_toolbox/maps/slam_toolbox_<name>.{pgm,yaml,posegraph,data}
#
# Reuse afterwards:
#   ros2 launch mapping_localization_pkg amcl_localization.launch.py \
#       map:=<printed path>.yaml
#   ros2 launch mapping_localization_pkg slam_toolbox.launch.py \
#       localization:=true map_file_name:=<printed path, no extension>

set -euo pipefail

NAME=""
SAVE_MAP=1
SAVE_GRAPH=1

usage() {
    echo "Usage: $0 [name] [--map-only | --graph-only]" >&2
    echo "  [name]         optional label; defaults to a timestamp" >&2
    echo "  --map-only     write only .pgm + .yaml   (for AMCL / nav2)" >&2
    echo "  --graph-only   write only .posegraph + .data (for slam_toolbox)" >&2
    echo "  default        writes all four" >&2
}

while [ $# -gt 0 ]; do
    case "$1" in
        --map-only)
            SAVE_GRAPH=0
            ;;
        --graph-only)
            SAVE_MAP=0
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            echo "Unknown option: $1" >&2
            usage
            exit 1
            ;;
        *)
            if [ -n "${NAME}" ]; then
                echo "Unexpected extra argument: $1" >&2
                usage
                exit 1
            fi
            NAME="$1"
            ;;
    esac
    shift
done

if [ "${SAVE_MAP}" -eq 0 ] && [ "${SAVE_GRAPH}" -eq 0 ]; then
    echo "--map-only and --graph-only are mutually exclusive." >&2
    exit 1
fi

NAME="${NAME:-$(date +%Y%m%d_%H%M%S)}"
MAPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../maps" && pwd)"
OUT_PREFIX="${MAPS_DIR}/slam_toolbox_${NAME}"

# Check every file we are about to write BEFORE writing any of them, so a
# collision on the second half never leaves a half-saved run behind.
EXISTING=()
CHECK_EXTS=()
if [ "${SAVE_MAP}" -eq 1 ]; then
    CHECK_EXTS+=(pgm yaml)
fi
if [ "${SAVE_GRAPH}" -eq 1 ]; then
    CHECK_EXTS+=(posegraph data)
fi
for ext in "${CHECK_EXTS[@]}"; do
    if [ -e "${OUT_PREFIX}.${ext}" ]; then
        EXISTING+=("${OUT_PREFIX}.${ext}")
    fi
done
if [ ${#EXISTING[@]} -gt 0 ]; then
    echo "Refusing to overwrite existing file(s):" >&2
    printf '  %s\n' "${EXISTING[@]}" >&2
    echo "Pick a different name, or move the old ones aside first." >&2
    exit 1
fi

# Liveness check for both halves, not just the pose graph: slam_toolbox
# advertises this service whenever it is up, so its absence means there is
# nothing to save from either way -- better than letting the map half sit in
# map_saver's 60s timeout to reach the same conclusion.
if ! ros2 service type /slam_toolbox/serialize_map >/dev/null 2>&1; then
    echo "/slam_toolbox/serialize_map not found -- is slam_toolbox.launch.py running" >&2
    echo "and active? Check with: ros2 lifecycle get /slam_toolbox" >&2
    exit 1
fi

# The pose graph goes first on purpose. It is the format that can detect a run
# with nothing in it (see the size guard below), and an empty graph means the
# occupancy grid would be empty too -- better to bail out before writing a
# misleading .pgm than after.
if [ "${SAVE_GRAPH}" -eq 1 ]; then
    echo "Serializing pose graph -> ${OUT_PREFIX}.posegraph / ${OUT_PREFIX}.data"

    # slam_toolbox appends the extensions itself, so pass the prefix.
    RESPONSE=$(ros2 service call /slam_toolbox/serialize_map \
        slam_toolbox/srv/SerializePoseGraph "{filename: '${OUT_PREFIX}'}")
    echo "${RESPONSE}"

    # result=0 is RESULT_SUCCESS; 255 is RESULT_FAILED_TO_WRITE_FILE. The
    # service call itself succeeds either way, so the response has to be
    # inspected.
    if ! grep -q "result=0" <<<"${RESPONSE}"; then
        echo "FAILED: slam_toolbox could not write the pose graph." >&2
        echo "Check that ${MAPS_DIR} exists and is writable." >&2
        exit 1
    fi

    if [ ! -e "${OUT_PREFIX}.posegraph" ]; then
        echo "WARNING: service reported success but ${OUT_PREFIX}.posegraph is missing." >&2
        exit 1
    fi

    # An EMPTY pose graph serializes "successfully" and then SEGFAULTS
    # localization_slam_toolbox_node when loaded back. That happens if you
    # serialize before any scan was ever processed -- no LiDAR, or slam_toolbox
    # was never actually fed /scan. Measured on this rover: an empty graph
    # writes a ~100-byte .data, a real one writes megabytes. Catch it here
    # rather than letting it crash at load time, where the error message says
    # nothing useful.
    DATA_BYTES=$(stat -c %s "${OUT_PREFIX}.data" 2>/dev/null || echo 0)
    if [ "${DATA_BYTES}" -lt 1024 ]; then
        echo >&2
        echo "WARNING: ${OUT_PREFIX}.data is only ${DATA_BYTES} bytes -- this pose" >&2
        echo "graph looks EMPTY. Loading it will segfault slam_toolbox." >&2
        echo "Was /scan actually being published and matched? Check:" >&2
        echo "  ros2 topic hz /scan" >&2
        echo "  ros2 topic echo /map --once --field info" >&2
        echo "Nothing usable was captured -- delete these and re-run after driving" >&2
        echo "the rover far enough to build a map." >&2
        exit 1
    fi
fi

if [ "${SAVE_MAP}" -eq 1 ]; then
    echo
    echo "Saving /map -> ${OUT_PREFIX}.pgm / ${OUT_PREFIX}.yaml"

    # map_saver_cli blocks until /map arrives. slam_toolbox only publishes it
    # once it has a map to publish, so without a bound this hangs forever on a
    # run that never saw a scan -- the same failure the size guard above
    # catches, just with no output at all to explain itself.
    #
    # Every argument is passed explicitly on purpose:
    #   -t     the topic. NOT --occ: that flag takes a NUMBER, so passing the
    #          topic to it silently parsed as 0.0 while the topic itself fell
    #          back to map_saver's default. Right answer, wrong reason -- it
    #          would have broken the moment /map was remapped.
    #   --occ  } thresholds. These end up in the .yaml and are what AMCL and
    #   --free } nav2's costmaps read back, so pin them rather than inheriting
    #          whatever this nav2 build happens to default to. 0.65/0.196 are
    #          the long-standing map_server values.
    #   --fmt  pgm, not png: greyscale, uncompressed, and what map_server has
    #          always assumed when a .yaml omits the format.
    if ! timeout 60 ros2 run nav2_map_server map_saver_cli \
            -f "${OUT_PREFIX}" -t /map --occ 0.65 --free 0.196 --fmt pgm; then
        echo "FAILED: could not save /map (timed out or map_saver errored)." >&2
        echo "Is slam_toolbox publishing? Check: ros2 topic echo /map --once --field info" >&2
        exit 1
    fi
fi

echo
echo "Saved:"
if [ "${SAVE_MAP}" -eq 1 ]; then
    echo "  ${OUT_PREFIX}.pgm / .yaml       (AMCL, nav2)"
fi
if [ "${SAVE_GRAPH}" -eq 1 ]; then
    echo "  ${OUT_PREFIX}.posegraph / .data (slam_toolbox)"
fi
echo
echo "Reuse with:"
if [ "${SAVE_MAP}" -eq 1 ]; then
    echo "  ros2 launch mapping_localization_pkg amcl_localization.launch.py \\"
    echo "      map:=${OUT_PREFIX}.yaml"
fi
if [ "${SAVE_GRAPH}" -eq 1 ]; then
    echo "  ros2 launch mapping_localization_pkg slam_toolbox.launch.py \\"
    echo "      localization:=true map_file_name:=${OUT_PREFIX}"
fi
