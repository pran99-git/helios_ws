# Copyright 2026
#
# Off-ground calibration helper for the A4WD3 mecanum wheel odometry.
#
# Reads both RoboClaws directly (NOT through ROS) and prints a live
# per-corner table of raw encoder counts + per-cycle deltas, plus the body
# twist the odometry kinematics would produce. Use it with the wheels OFF
# the ground to confirm wheel signs and directions.
#
# Wiring (final): ACM0 = LEFT controller (M2 front, M1 rear), ACM1 = RIGHT
# controller (M1 front, M2 rear).
#
# IMPORTANT: a serial port cannot be opened twice. roboclaw_driver_node is
# the sole owner of the RoboClaw serial ports -- stop it before running this.
#
# Usage:
#   ros2 run low_level_control_pkg wheel_monitor
#   ros2 run low_level_control_pkg wheel_monitor --baud 38400
#
# Spin one wheel at a time in its FORWARD-drive direction and watch which corner
# moves and with what sign. Every wheel should read POSITIVE when driven
# forward; a negative corner -> set its invert_* flag true in the YAML.

import argparse
import math
import sys
import time

from teleop.mecanum_kinematics import CORNERS
from teleop.roboclaw_driver import RoboClaw, SerialBus


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Live RoboClaw encoder monitor for mecanum odometry calibration."
    )
    # Use the stable udev symlinks (NOT /dev/ttyACM*, which renumber by USB
    # insertion order) so the monitor and the node always agree on left/right.
    p.add_argument("--left-port", default="/dev/roboclaw_left")
    p.add_argument("--right-port", default="/dev/roboclaw_right")
    p.add_argument("--left-address", type=lambda x: int(x, 0), default=0x80)
    p.add_argument("--right-address", type=lambda x: int(x, 0), default=0x80)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--rate", type=float, default=10.0, help="poll rate (Hz)")
    # Geometry (matches the node defaults) for the computed twist read-out.
    p.add_argument("--wheel-radius", type=float, default=0.076)
    p.add_argument("--counts-per-rev", type=float, default=2448.0)
    p.add_argument("--wheelbase", type=float, default=0.220)
    p.add_argument("--track-width", type=float, default=0.330)
    # strip ROS args (e.g. --ros-args) that ros2 run may append
    known, _ = p.parse_known_args(argv)
    return known


def fmt(delta):
    """Format a per-cycle delta with a direction marker."""
    if delta > 2:
        arrow = "+"
    elif delta < -2:
        arrow = "-"
    else:
        arrow = " "
    active = " <==" if abs(delta) > 2 else "    "
    return f"{arrow}{delta:+8d}{active}"


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)

    buses = {}

    def get_bus(port):
        if port not in buses:
            buses[port] = SerialBus(port, args.baud, timeout=0.1)
        return buses[port]

    try:
        left = RoboClaw(get_bus(args.left_port), args.left_address)
        right = RoboClaw(get_bus(args.right_port), args.right_address)
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR opening serial port: {exc}", file=sys.stderr)
        print(
            "Is roboclaw_driver_node still running? Stop it first "
            "(a serial port can only be opened once).",
            file=sys.stderr,
        )
        return 1

    meters_per_count = (2.0 * math.pi * args.wheel_radius) / args.counts_per_rev
    l_sum = 0.5 * (args.wheelbase + args.track_width)
    period = 1.0 / args.rate

    prev = None
    prev_t = None
    n_lines = 0

    print("RoboClaw encoder monitor — spin one wheel FORWARD at a time.")
    print("Every wheel should read POSITIVE (+) when driven forward.")
    print("A negative (-) corner -> set its invert_* flag true in the YAML.")
    print(
        f"ACM0/left={args.left_port}@{hex(args.left_address)}  "
        f"ACM1/right={args.right_port}@{hex(args.right_address)}  baud={args.baud}"
    )
    print("Ctrl-C to quit.\n")

    def read_corners():
        """Return {corner: count or None}. Fixed wiring: ACM0 left, ACM1 right;
        left front=M2/rear=M1, right front=M1/rear=M2."""
        lr = left.read_encoders()  # (M1, M2) or None
        rr_ = right.read_encoders()
        return {
            "front_left": lr[1] if lr else None,  # left M2
            "rear_left": lr[0] if lr else None,  # left M1
            "front_right": rr_[0] if rr_ else None,  # right M1
            "rear_right": rr_[1] if rr_ else None,  # right M2
        }

    try:
        while True:
            t = time.time()
            corners = read_corners()

            lines = []
            if prev is None or any(v is None for v in corners.values()):
                for name in CORNERS:
                    val = corners[name]
                    shown = "READ FAIL" if val is None else f"{val}"
                    lines.append(f"  {name:12s} count={shown:>12}   delta=   (seeding)")
                twist_line = "  twist: (waiting for valid reads)"
            else:
                dt = max(t - prev_t, 1e-3)
                deltas = {k: (corners[k] - prev[k]) for k in corners}
                for name in CORNERS:
                    lines.append(
                        f"  {name:12s} count={corners[name]:>12d}   "
                        f"delta={fmt(deltas[name])}"
                    )
                # Body twist from the mecanum kinematics (uninverted).
                d = {k: deltas[k] * meters_per_count for k in deltas}
                vx = (
                    0.25
                    * (
                        d["front_left"]
                        + d["front_right"]
                        + d["rear_left"]
                        + d["rear_right"]
                    )
                    / dt
                )
                vy = (
                    0.25
                    * (
                        -d["front_left"]
                        + d["front_right"]
                        + d["rear_left"]
                        - d["rear_right"]
                    )
                    / dt
                )
                wz = (
                    (
                        -d["front_left"]
                        + d["front_right"]
                        - d["rear_left"]
                        + d["rear_right"]
                    )
                    / (4.0 * l_sum)
                ) / dt
                twist_line = (
                    f"  twist (no inversion):  "
                    f"vx={vx:+.3f} m/s  vy={vy:+.3f} m/s  wz={wz:+.3f} rad/s"
                )

            prev = corners if all(v is not None for v in corners.values()) else prev
            prev_t = t

            out = "\n".join(lines + ["", twist_line])
            # Redraw in place.
            if n_lines:
                sys.stdout.write(f"\033[{n_lines}A")
            sys.stdout.write("\033[J" + out + "\n")
            sys.stdout.flush()
            n_lines = len(lines) + 2

            sleep = period - (time.time() - t)
            if sleep > 0:
                time.sleep(sleep)
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        for bus in buses.values():
            bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
