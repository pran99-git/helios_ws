#!/usr/bin/env python3
"""Time how long AMCL takes to converge, so tuning changes get a number.

Watches /amcl_pose and reports the wall time from "lost" to "converged",
using the same thresholds amcl_recovery itself uses, so the number here
matches the moment recovery would declare success.

Run it BEFORE launching AMCL, drive or let recovery drive, and read the
summary on Ctrl+C:

    python3 amcl_convergence.py
    python3 amcl_convergence.py --converged-std 0.20 --csv run_a.csv

Change one parameter in amcl.yaml, re-run, compare. Tuning several knobs at
once is untraceable: only a measured time-to-converge shows which of them
earned its place.
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

# Row-major 6x6 over [x, y, z, roll, pitch, yaw].
COV_XX, COV_YY, COV_YAW = 0, 7, 35


class ConvergenceTimer(Node):
    """Reports time-to-converge from AMCL's reported covariance."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("amcl_convergence")
        self._converged_std = args.converged_std
        self._converged_yaw = args.converged_yaw
        self._csv = csv.writer(args.csv) if args.csv else None
        if self._csv:
            self._csv.writerow(["elapsed_s", "std_x_m", "std_y_m", "std_yaw_rad"])

        self._t0: float | None = None
        # AMCL publishes once when seeded or scattered, then only after
        # update_min_d/_a of motion. So the SECOND pose is the first the rover
        # earned by moving, and timing from there excludes however long it sat
        # still before the driver started. Timing from the first pose made a
        # 28 s convergence read as 237 s.
        self._t_motion: float | None = None
        self._converged_at: float | None = None
        self._updates = 0
        self._best = math.inf

        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10
        )
        print(
            f"waiting for /amcl_pose  (converged = position std < "
            f"{self._converged_std} m and yaw std < {self._converged_yaw} rad)",
            flush=True,
        )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        now = time.time()
        if self._t0 is None:
            self._t0 = now
        elif self._t_motion is None:
            self._t_motion = now
        elapsed = now - self._t0
        self._updates += 1

        cov = msg.pose.covariance
        sx, sy = math.sqrt(cov[COV_XX]), math.sqrt(cov[COV_YY])
        syaw = math.sqrt(cov[COV_YAW])
        position = math.hypot(sx, sy)
        self._best = min(self._best, position)

        if self._csv:
            self._csv.writerow(
                [f"{elapsed:.2f}", f"{sx:.4f}", f"{sy:.4f}", f"{syaw:.4f}"]
            )

        mark = ""
        if (
            self._converged_at is None
            and position < self._converged_std
            and syaw < self._converged_yaw
        ):
            self._converged_at = now - (self._t_motion or self._t0)
            mark = "   <<< CONVERGED"
        print(
            f"{elapsed:7.1f}s  update {self._updates:4d}  "
            f"std x={sx:5.2f} y={sy:5.2f} yaw={syaw:5.2f}{mark}",
            flush=True,
        )

    def report(self) -> None:
        """Prints the summary. Safe to call even if nothing ever arrived.

        Every print flushes: this runs during shutdown, and a buffered stdout
        pipe would swallow the one number the tool exists to produce.
        """
        out = []
        out.append("\n--- summary " + "-" * 50)
        if self._t0 is None:
            out.append(
                "no /amcl_pose received. Was AMCL running, and did the rover move?"
            )
            out.append(
                "AMCL only publishes after update_min_d / update_min_a of motion."
            )
        else:
            total = time.time() - self._t0
            idle = (self._t_motion - self._t0) if self._t_motion else 0.0
            driving = max(total - idle, 1e-6)
            out.append(f"updates seen        : {self._updates}")
            out.append(f"idle before motion  : {idle:.1f} s (excluded)")
            out.append(f"update rate driving : {self._updates / driving:.2f} Hz")
            out.append(f"best position std   : {self._best:.2f} m")
            if self._converged_at is not None:
                out.append(
                    f"TIME TO CONVERGE    : {self._converged_at:.1f} s "
                    f"(from first motion)"
                )
            else:
                out.append(f"TIME TO CONVERGE    : never (drove {driving:.1f} s)")
        print("\n".join(out), flush=True)


def main() -> None:
    """Parses arguments and runs the timer until interrupted."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--converged-std",
        type=float,
        default=0.25,
        help="m, position std below this counts as converged "
        "(matches amcl_recovery's max_position_std)",
    )
    parser.add_argument(
        "--converged-yaw",
        type=float,
        default=0.15,
        help="rad, yaw std below this counts as converged",
    )
    parser.add_argument(
        "--csv",
        type=argparse.FileType("w"),
        default=None,
        help="also write the trace here, for plotting two runs",
    )
    args = parser.parse_args()

    # rclpy's own signal handling tears the context down before any exception
    # reaches us, so the summary never prints. Same problem, and the same fix,
    # as amcl_recovery_node's main(): handle the signal here and leave the
    # context alive long enough to report.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ConvergenceTimer(args)
    stopping = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.report()
        if args.csv:
            args.csv.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
