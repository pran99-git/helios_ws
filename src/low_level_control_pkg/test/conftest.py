"""Makes `teleop` importable without a colcon build.

The package root (src/low_level_control_pkg) is one level up from this file.
Without this, `pytest src/low_level_control_pkg/test` only works when an
install overlay happens to be sourced -- and in this workspace the sourced
overlay can point at a different worktree entirely.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
