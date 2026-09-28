"""Single point of coupling to the OmniVLA submodule.

Everything this package borrows from vendor code is re-exported here, so the
submodule's layout is referenced in exactly one file.
"""

from __future__ import annotations

import pathlib
import sys

VENDOR_INFERENCE = pathlib.Path(__file__).resolve().parents[1] / "omnivla" / "inference"

# utils_policy uses bare imports ("from model_omnivla_edge import ..."), which
# only resolve with the vendor's own directory on sys.path. The vendor script
# gets this for free because Python adds a script's directory; we do not.
if str(VENDOR_INFERENCE) not in sys.path:
    sys.path.insert(0, str(VENDOR_INFERENCE))

from utils_policy import (
    load_model,
    transform_images_map,
    transform_images_PIL_mask,
)

__all__ = [
    "VENDOR_INFERENCE",
    "load_model",
    "transform_images_PIL_mask",
    "transform_images_map",
]
