"""The deployment contract, re-exported for the training venv.

The observation layout, the action scaling and the velocity envelope live in
`rl_bridge`, the package that ships on the rover, so that the robot's code
depends on nothing outside its own install. The simulator is therefore the
side that reaches across, exactly as `envs/route.py` reaches into
`vla_bridge` for route geometry.

Training and deployment then run the SAME assembly function. The alternative
is two copies of a 73-value layout that must agree forever, where a
disagreement is silent: the policy keeps returning plausible twists, it is
just answering a different question.

This module exists for the path setup. `rl_sim` runs under `.venvs/rl_train`
with PYTHONPATH stripped, so neither sibling package is importable until the
two inserts below have happened.
"""

from __future__ import annotations

import sys
from pathlib import Path

#: src/rl_pkg/rl_bridge and src/vla_pkg/vla_bridge, the two ROS packages whose
#: ROS-free modules the simulator shares.
RL_BRIDGE = Path(__file__).resolve().parents[2] / "rl_bridge"
VLA_BRIDGE = Path(__file__).resolve().parents[3] / "vla_pkg" / "vla_bridge"

for package in (RL_BRIDGE, VLA_BRIDGE):
    if not package.is_dir():  # pragma: no cover - a broken checkout
        raise ImportError(f"{package.name} not found at {package}")
    if str(package) not in sys.path:
        sys.path.insert(0, str(package))

from rl_bridge.contract import (
    I_CARROTS,
    I_CMD,
    I_MEAS,
    I_REMAINING,
    I_SCAN,
    LOOKAHEADS,
    MAX_RANGE,
    MIN_RANGE,
    OBS_DIM,
    SCAN_BEAMS,
    SCAN_FOV,
    YAW_FEEDBACK_CHANNELS,
    Observation,
    ObservationContract,
    assemble,
    blind,
    clean,
    downsample,
    scale_action,
    sector,
)
from rl_bridge.limits import (
    DEFAULT_ACCEL,
    DEFAULT_DECEL,
    RateLimiter,
    VelocityLimits,
)

__all__ = [
    "DEFAULT_ACCEL",
    "DEFAULT_DECEL",
    "I_CARROTS",
    "I_CMD",
    "I_MEAS",
    "I_REMAINING",
    "I_SCAN",
    "LOOKAHEADS",
    "MAX_RANGE",
    "MIN_RANGE",
    "OBS_DIM",
    "SCAN_BEAMS",
    "SCAN_FOV",
    "YAW_FEEDBACK_CHANNELS",
    "Observation",
    "ObservationContract",
    "RateLimiter",
    "VelocityLimits",
    "assemble",
    "blind",
    "clean",
    "downsample",
    "scale_action",
    "sector",
]
