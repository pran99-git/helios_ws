"""Reduce the helios_description CAD meshes to something MuJoCo will load.

MuJoCo's STL reader rejects any mesh over 200,000 faces. The CAD exports run
from 4 k to 1.66 M, so three of the five need reducing before helios.xml can
open at all.

Run from the rl_sim directory, with the rl_train venv:

    env -u PYTHONPATH ../../../.venvs/rl_train/bin/python tools/decimate_meshes.py

Output lands in assets/meshes/ and is gitignored: it is derived from
helios_description, which stays the one source of truth for the geometry.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import trimesh

#: MuJoCo's own ceiling is 200 k faces. 60 k is far below it and still finer
#: than anything visible at this robot's on-screen size.
TARGET_FACES = 60_000

SRC = Path(__file__).resolve().parents[3] / "helios_description" / "meshes"
DST = Path(__file__).resolve().parent.parent / "assets" / "meshes"


def decimate(path: Path, target: int) -> tuple[trimesh.Trimesh, int, int]:
    """Weld and simplify one mesh.

    STL stores every triangle with its own copy of each vertex, so a mesh
    loaded straight from disk has no shared edges for quadric decimation to
    collapse along. Welding first is what makes the reduction work: without
    it the chassis stalls at 461 k faces, still over MuJoCo's limit.

    Args:
        path: Source STL.
        target: Face count to aim for. Meshes already under it are untouched.

    Returns:
        The mesh, its original face count, and its final face count.
    """
    import fast_simplification as fs

    mesh = trimesh.load_mesh(path, process=False)
    before = len(mesh.faces)
    if before <= target:
        return mesh, before, before

    mesh.merge_vertices()
    verts, faces = fs.simplify(
        np.asarray(mesh.vertices, np.float32),
        np.asarray(mesh.faces, np.int32),
        target_count=target,
    )
    reduced = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
    return reduced, before, len(reduced.faces)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--target", type=int, default=TARGET_FACES, help="target face count"
    )
    args = ap.parse_args()

    if not SRC.is_dir():
        print(f"source meshes not found at {SRC}", file=sys.stderr)
        return 1
    DST.mkdir(parents=True, exist_ok=True)

    total_before = total_after = 0
    for src in sorted(SRC.glob("*.STL")):
        t0 = time.perf_counter()
        mesh, before, after = decimate(src, args.target)
        dst = DST / src.name
        mesh.export(dst)
        total_before += src.stat().st_size
        total_after += dst.stat().st_size
        note = "kept" if before == after else f"{before:,} ->"
        print(
            f"  {src.name:30s} {note:>12s} {after:>7,} faces  "
            f"{src.stat().st_size / 1e6:6.1f} -> {dst.stat().st_size / 1e6:4.1f} MB  "
            f"{time.perf_counter() - t0:4.1f}s"
        )

    print(f"\n{total_before / 1e6:.1f} MB -> {total_after / 1e6:.1f} MB in {DST}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
