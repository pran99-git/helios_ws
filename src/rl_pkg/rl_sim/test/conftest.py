"""Makes `envs` importable without setting PYTHONPATH.

The package root (src/rl_pkg/rl_sim) is one level up from this file. Without
this, running pytest against this directory needs PYTHONPATH=. on every
invocation, which is easy to forget and silently picks up a different tree
when a ROS overlay is sourced.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
