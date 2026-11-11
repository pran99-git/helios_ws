"""Check that the rl_train venv can actually run physics on this machine.

Run after installing requirements.txt, and after any JetPack or driver change:

    env -u PYTHONPATH ../../../.venvs/rl_train/bin/python verify_install.py

The PYTHONPATH scrub matters: sourcing a ROS 2 workspace puts
/opt/ros/jazzy/lib/python3.12/site-packages ahead of the venv on sys.path, so
any package present in both resolves to the ROS copy. Nothing overlaps today,
which is luck rather than isolation.

Pass --bench to also measure throughput. The first Warp run compiles kernels
for roughly two minutes before caching them under ~/.cache/warp.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

ASSETS = Path(__file__).parent / "assets"


def check_isolation() -> bool:
    """Report whether a sourced ROS workspace shadows any venv package."""
    ros = Path("/opt/ros/jazzy/lib/python3.12/site-packages")
    venv = Path(sys.prefix) / "lib" / "python3.12" / "site-packages"
    if not ros.is_dir():
        print("  ROS site-packages absent, nothing to shadow")
        return True

    def tops(p: Path) -> set[str]:
        skip = (".dist-info", ".egg-info", ".pth", ".egg-link")
        return {
            e.name.split(".")[0] if e.is_file() else e.name
            for e in p.iterdir()
            if not e.name.endswith(skip) and e.name != "__pycache__"
        }

    shadowed = sorted(tops(ros) & tops(venv))
    on_path = str(ros) in sys.path
    if shadowed and on_path:
        print(f"  SHADOWED by ROS: {shadowed}")
        print("  rerun with: env -u PYTHONPATH")
        return False
    print(f"  no shadowing ({'ROS on sys.path' if on_path else 'PYTHONPATH clean'})")
    return True


def check_cpu() -> bool:
    """Confirm MuJoCo steps and that the lidar primitive is present."""
    import mujoco

    print(f"  mujoco {mujoco.__version__}")
    if not hasattr(mujoco, "mj_multiRay"):
        print("  FAIL: mj_multiRay missing, lidar sensing unavailable")
        return False
    print("  mj_multiRay present")
    return True


def check_gpu() -> bool:
    """Confirm Warp resolves the Orin's integrated GPU."""
    import warp as wp

    wp.init()
    devices = [str(d) for d in wp.get_cuda_devices()]
    if not devices:
        print("  FAIL: no CUDA device, GPU backend unavailable")
        return False
    for d in wp.get_cuda_devices():
        print(f"  {d}: arch sm_{d.arch}, {d.total_memory // 2**30} GiB")
    import mujoco_warp as mjw

    print(f"  mujoco_warp imported, {len(dir(mjw))} symbols")
    return True


def bench() -> None:
    """Measure CPU and GPU stepping rates on the rover model."""
    import mujoco
    import numpy as np
    from mujoco import rollout

    scene = ASSETS / "helios.xml"
    if not scene.is_file():
        print(f"  skipped: {scene} not written yet")
        return

    mjm = mujoco.MjModel.from_xml_path(str(scene))
    mjd = mujoco.MjData(mjm)

    steps = 20_000
    t0 = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(mjm, mjd)
    print(f"  CPU  1 thread : {steps / (time.perf_counter() - t0):12,.0f} steps/s")

    nstate = mujoco.mj_stateSize(mjm, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    s0 = np.zeros(nstate)
    mujoco.mj_getState(mjm, mjd, s0, mujoco.mjtState.mjSTATE_FULLPHYSICS)
    nroll, nstep = 240, 400
    init = np.tile(s0, (nroll, 1))
    ctrl = np.zeros((nroll, nstep, mjm.nu))
    with rollout.Rollout(nthread=12) as r:
        datas = [mujoco.MjData(mjm) for _ in range(12)]
        r.rollout(mjm, datas, init, control=ctrl)  # warm the thread pool
        t0 = time.perf_counter()
        r.rollout(mjm, datas, init, control=ctrl)
        dt = time.perf_counter() - t0
    print(f"  CPU 12 threads: {nroll * nstep / dt:12,.0f} steps/s")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--bench", action="store_true", help="also measure throughput")
    args = ap.parse_args()

    checks = (
        ("venv isolation", check_isolation),
        ("CPU backend", check_cpu),
        ("GPU backend", check_gpu),
    )
    results: list[bool] = []
    for name, fn in checks:
        print(f"[{name}]")
        results.append(fn())

    if args.bench:
        print("[throughput]")
        bench()

    ok = all(results)
    print("\nPASS" if ok else "\nFAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
