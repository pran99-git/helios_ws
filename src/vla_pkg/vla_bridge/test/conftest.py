"""Makes `vla_bridge` importable without a colcon build.

The package root (src/vla_pkg/vla_bridge) is one level up from this file.
Without this, running pytest against this directory only works when an install
overlay happens to be sourced.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
