"""Makes `rl_bridge` importable without setting PYTHONPATH.

The package root (src/rl_pkg/rl_bridge) is one level up from this file.
Mirrors rl_sim/test/conftest.py: without it, pytest needs PYTHONPATH on
every invocation and silently picks a different tree under a ROS overlay.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
