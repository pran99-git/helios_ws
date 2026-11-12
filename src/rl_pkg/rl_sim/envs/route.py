"""Path-following geometry, borrowed from vla_bridge.

`vla_bridge/subgoal.py` already solves this: project a pose onto a polyline,
measure what is left, and pick a carrot a fixed distance ahead. It states in
its own docstring that it imports no ROS, and `goal_resolution` beside it is
stdlib only, so both run unchanged under the training venv.

Importing it rather than copying keeps ONE definition of route geometry for
the robot and the simulator, which matters because the RL policy is meant to
follow the same routes `subgoal_walker_node` walks. A second copy would drift.

The cost is that training now depends on a ROS package's source layout. That
coupling is confined to this module, and `test_route.py` fails loudly if the
import breaks or if ROS leaks into those files.
"""

from __future__ import annotations

import sys
from pathlib import Path

#: src/vla_pkg/vla_bridge, which is where the `vla_bridge` package lives.
VLA_BRIDGE = Path(__file__).resolve().parents[3] / "vla_pkg" / "vla_bridge"

if not VLA_BRIDGE.is_dir():  # pragma: no cover - a broken checkout
    raise ImportError(f"vla_bridge not found at {VLA_BRIDGE}")
if str(VLA_BRIDGE) not in sys.path:
    sys.path.insert(0, str(VLA_BRIDGE))

from vla_bridge.goal_resolution import Pose2D, wrap_angle
from vla_bridge.subgoal import (
    Projection,
    carrot,
    heading_error,
    point_at_arc,
    project,
    remaining,
    route_length,
)

__all__ = [
    "Pose2D",
    "Projection",
    "carrot",
    "heading_error",
    "point_at_arc",
    "project",
    "remaining",
    "route_length",
    "wrap_angle",
]
