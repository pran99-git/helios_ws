#!/usr/bin/env bash
# Extract a finished RTAB-Map run into the two STANDALONE formats that other
# tools consume. Neither is produced automatically.
#
# READ THIS FIRST -- the run is already saved without you doing anything.
# Ctrl+C on rtabmap.launch.py writes the whole SLAM session to its .db:
#     rtabmap: Saving database/long-term memory... done! (..., 364 MB)
#     rtabmap: 2D occupancy grid map saved.
# That database holds BOTH maps -- the pose graph, every keyframe's RGB+depth,
# the laser scans, and the occupancy grid. Nothing is lost by never running
# this script. It exists only because a .db is not a format that AMCL, nav2,
# CloudCompare or Meshlab can open.
#
# So: kill rtabmap with Ctrl+C, never `kill -9`. A SIGKILL skips the save and
# the run is gone.
#
#   <prefix>.pgm + <prefix>.yaml   the 2D occupancy grid, image + metadata.
#                                  What AMCL and nav2 costmaps read.
#                                  *** MUST be captured while rtabmap is still
#                                  RUNNING *** -- it is a live capture of the
#                                  /rtabmap/map topic. There is no offline
#                                  exporter for it: rtabmap-export has no grid
#                                  option, and the databaseViewer's equivalent
#                                  is GUI-only.
#
#   <prefix>_cloud.ply             the assembled 3D point cloud, exported from
#                                  the .db AFTER the run. Opens in CloudCompare
#                                  or Meshlab.
#
# The two halves therefore have OPPOSITE timing requirements, which is why this
# script checks what is running and tells you which half it can do:
#
#   BEFORE Ctrl+C:   save_rtabmap.sh my_run --map-only     # 2D grid
#   AFTER  Ctrl+C:   save_rtabmap.sh my_run --cloud-only   # 3D cloud
#   both, in order:  run it twice, same name
#
# Run directly from source -- it locates maps/ relative to its own path:
#   S=~/helios_ws/src/mapping_localization_pkg/rtabmap/scripts
#   $S/save_rtabmap.sh                       # auto-name, does what it can
#   $S/save_rtabmap.sh lab_run --map-only
#   $S/save_rtabmap.sh lab_run --cloud-only
#   $S/save_rtabmap.sh lab_run --cloud-only --db <path>   # a specific database
#
# Output: rtabmap/maps/rtabmap_<name>.{pgm,yaml}  and  rtabmap_<name>_cloud.ply

set -euo pipefail

NAME=""
DB=""
DO_MAP=1
DO_CLOUD=1

usage() {
    cat >&2 <<'EOF'
Usage: save_rtabmap.sh [name] [--map-only | --cloud-only] [--db PATH]
  [name]         optional label; defaults to a timestamp
  --map-only     only .pgm + .yaml   (needs rtabmap RUNNING)
  --cloud-only   only _cloud.ply     (needs rtabmap STOPPED)
  --db PATH      database to export from (default: newest in maps/)
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --map-only)   DO_CLOUD=0 ;;
        --cloud-only) DO_MAP=0 ;;
        --db)         shift; [ $# -gt 0 ] || { echo "--db needs a path" >&2; exit 1; }; DB="$1" ;;
        -h|--help)    usage; exit 0 ;;
        -*)           echo "Unknown option: $1" >&2; usage; exit 1 ;;
        *)
            if [ -n "${NAME}" ]; then
                echo "Unexpected extra argument: $1" >&2; usage; exit 1
            fi
            NAME="$1" ;;
    esac
    shift
done

if [ "${DO_MAP}" -eq 0 ] && [ "${DO_CLOUD}" -eq 0 ]; then
    echo "--map-only and --cloud-only are mutually exclusive." >&2
    exit 1
fi

NAME="${NAME:-$(date +%Y%m%d_%H%M%S)}"
MAPS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../maps" && pwd)"
OUT_PREFIX="${MAPS_DIR}/rtabmap_${NAME}"

# Is rtabmap up? This decides which half is even possible, so check once and
# reuse the answer rather than letting each half fail in its own way.
RUNNING=0
if ros2 node list 2>/dev/null | grep -qx '/rtabmap/rtabmap'; then
    RUNNING=1
fi

# ---------------------------------------------------------------- 2D grid ---
if [ "${DO_MAP}" -eq 1 ]; then
    if [ "${RUNNING}" -eq 0 ]; then
        echo "SKIPPING the 2D grid: /rtabmap/rtabmap is not running." >&2
        echo "It is a live capture of /rtabmap/map -- start the run again and" >&2
        echo "save BEFORE Ctrl+C, or open the .db in rtabmap-databaseViewer." >&2
        [ "${DO_CLOUD}" -eq 0 ] && exit 1
    else
        for ext in pgm yaml; do
            if [ -e "${OUT_PREFIX}.${ext}" ]; then
                echo "Refusing to overwrite ${OUT_PREFIX}.${ext}" >&2
                exit 1
            fi
        done
        echo "Saving /rtabmap/map -> ${OUT_PREFIX}.pgm / .yaml"
        # Bounded, and every argument explicit -- same reasoning as
        # slam_toolbox/scripts/save_slam.sh. 0.65/0.196 are the long-standing
        # map_server thresholds that AMCL and nav2 costmaps read back out of
        # the .yaml; pgm because map_server assumes it when format is omitted.
        # Note -t: --occ takes a NUMBER, so passing a topic to it parses as 0.0
        # and silently falls back to the default topic.
        if ! timeout 60 ros2 run nav2_map_server map_saver_cli \
                -f "${OUT_PREFIX}" -t /rtabmap/map --occ 0.65 --free 0.196 --fmt pgm; then
            echo "FAILED: could not save /rtabmap/map." >&2
            echo "Is rtabmap actually processing? Zero /rtabmap/info messages means" >&2
            echo "its 3-way sync never completes -- usually a dead /scan. Check:" >&2
            echo "  ros2 topic hz /scan" >&2
            echo "  ros2 topic hz /rtabmap/info" >&2
            exit 1
        fi
    fi
fi

# --------------------------------------------------------------- 3D cloud ---
if [ "${DO_CLOUD}" -eq 1 ]; then
    if [ "${RUNNING}" -eq 1 ]; then
        echo >&2
        echo "SKIPPING the 3D cloud: rtabmap is still running." >&2
        echo "Its database is open and the final save has not happened yet, so an" >&2
        echo "export now would miss the end of the run. Ctrl+C first, then:" >&2
        echo "  $0 ${NAME} --cloud-only" >&2
        exit 0
    fi

    if [ -z "${DB}" ]; then
        DB=$(ls -t "${MAPS_DIR}"/*.db 2>/dev/null | head -1 || true)
    fi
    if [ -z "${DB}" ] || [ ! -e "${DB}" ]; then
        echo "No database found in ${MAPS_DIR} -- pass one with --db." >&2
        exit 1
    fi

    echo
    echo "Exporting cloud from $(basename "${DB}") -> ${OUT_PREFIX}_cloud.ply"

    # --max_range 4 matches the depth range RTAB-Map itself trusts on a 0.120 m
    # baseline; stereo error grows as Z^2 and is ~0.8 m at 10 m, so points
    # beyond a few metres are noise that makes the cloud look worse, not
    # bigger. --decimation 1 + --voxel 0.01 are FULL detail: this runs offline
    # with no Jetson budget to respect, unlike the live RViz MapCloud which is
    # deliberately coarser. --noise_radius/--noise_k strip the stereo flying
    # pixels that the mapping config leaves in on purpose (tightening
    # depth_confidence to remove them cost 5x the visual loop closures on
    # 2026-08-24 -- clean the cloud here, never in the sensor config).
    # --output takes the STEM only: rtabmap-export appends "_cloud" itself, so
    # passing "..._cloud" here produced rtabmap_<name>_cloud_cloud.ply while
    # this script reported a path that did not exist. Verified 2026-08-28.
    rtabmap-export \
        --cloud \
        --output "rtabmap_${NAME}" \
        --output_dir "${MAPS_DIR}" \
        --max_range 4 \
        --decimation 1 \
        --voxel 0.01 \
        --noise_radius 0.05 \
        --noise_k 5 \
        "${DB}"
fi

echo
echo "Saved:"
[ "${DO_MAP}" -eq 1 ] && [ "${RUNNING}" -eq 1 ] && echo "  ${OUT_PREFIX}.pgm / .yaml        (AMCL, nav2)"
[ "${DO_CLOUD}" -eq 1 ] && [ "${RUNNING}" -eq 0 ] && echo "  ${OUT_PREFIX}_cloud.ply         (CloudCompare, Meshlab)"
echo "  the .db itself is the archive -- it holds both maps and is the only"
echo "  format rtabmap can resume or re-optimize from."
