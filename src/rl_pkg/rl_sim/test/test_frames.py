"""Guards the body-frame action space.

This exists because the frame was silently wrong once. With the slide joints
listed before the hinge in helios.xml, MuJoCo translates in the WORLD frame:
a commanded "forward" moved +x no matter which way the rover faced. The
observation is body-frame and carries no absolute yaw, so nothing connected
the two and the task was not learnable at all. A 3.5 M step run reached the
goal zero times.

Nothing about that is visible in a reward curve, so it is pinned here.
"""

from __future__ import annotations

import mujoco
import numpy as np
import pytest
from envs import scenarios
from envs.rover_env import RoverEnv


@pytest.fixture
def model() -> mujoco.MjModel:
    return scenarios.build(scenarios.Scenario(bounds=(8.0, 8.0)), ())


def addresses(model: mujoco.MjModel) -> dict[str, int]:
    return {
        name: int(
            model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        )
        for name in ("hinge_z", "slide_x", "slide_y")
    }


def test_hinge_is_declared_before_the_slides(model: mujoco.MjModel) -> None:
    """Joint ORDER is what puts the slides in the body frame."""
    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        for j in range(model.njnt)
    ]
    assert names.index("hinge_z") < names.index("slide_x")
    assert names.index("hinge_z") < names.index("slide_y")


@pytest.mark.parametrize(
    ("yaw_deg", "axis", "sign"),
    [(0.0, 0, +1), (90.0, 1, +1), (180.0, 0, -1), (-90.0, 1, -1)],
)
def test_forward_follows_the_heading(
    model: mujoco.MjModel, yaw_deg: float, axis: int, sign: int
) -> None:
    """Commanding vx must move the rover along its own nose, not world +x."""
    data = mujoco.MjData(model)
    adr = addresses(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    data.qpos[adr["hinge_z"]] = np.deg2rad(yaw_deg)
    mujoco.mj_forward(model, data)
    start = data.xpos[body][:2].copy()

    data.ctrl[:] = [0.40, 0.0, 0.0]
    for _ in range(400):
        mujoco.mj_step(model, data)

    moved = data.xpos[body][:2] - start
    assert sign * moved[axis] > 0.3, f"expected travel along axis {axis}"
    assert abs(moved[1 - axis]) < 0.05, "should not drift sideways"


def test_strafe_is_perpendicular_to_the_nose(model: mujoco.MjModel) -> None:
    data = mujoco.MjData(model)
    adr = addresses(model)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    data.qpos[adr["hinge_z"]] = np.deg2rad(90.0)
    mujoco.mj_forward(model, data)
    start = data.xpos[body][:2].copy()

    data.ctrl[:] = [0.0, 0.10, 0.0]
    for _ in range(600):
        mujoco.mj_step(model, data)

    moved = data.xpos[body][:2] - start
    # Facing +y, a left strafe goes to world -x.
    assert moved[0] < -0.1
    assert abs(moved[1]) < 0.05


def test_env_reports_world_position_not_joint_coordinates() -> None:
    """`xpos`, never the slide qpos.

    With the hinge first the slide coordinates are displacements along a
    ROTATED axis. Reading them as world x and y made the rover appear to fly
    off the map while it was really driving in circles.
    """
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(8.0, 8.0)), seed=0)
    env.reset(seed=0)
    adr = addresses(env.model)
    body = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    for _ in range(50):
        env.step(np.array([1.0, 0.0, 0.3], dtype=np.float32))

    reported = env._true_pose()
    assert reported[:2] == pytest.approx(env.data.xpos[body][:2], abs=1e-9)
    # And it must NOT be the raw joint values, once yaw is non-zero.
    joints = np.array([env.data.qpos[adr["slide_x"]], env.data.qpos[adr["slide_y"]]])
    assert not np.allclose(reported[:2], joints, atol=1e-3)


def test_start_pose_lands_where_it_was_asked_to() -> None:
    """Setting a start has to rotate INTO the joint frame."""
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(8.0, 8.0)), seed=1)
    for seed in range(20):
        env.reset(seed=seed)
        wanted = env.route[0]
        got = env._true_pose()
        assert got[0] == pytest.approx(wanted.x, abs=1e-6)
        assert got[1] == pytest.approx(wanted.y, abs=1e-6)


def test_measured_twist_is_body_frame() -> None:
    """qvel on the slides is already body twist, which the observation wants."""
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(8.0, 8.0)), seed=2)
    env.reset(seed=3)
    for _ in range(80):
        env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))
    twist = env._true_twist()
    assert twist[0] > 0.2, "forward command should show as forward twist"
    assert abs(twist[1]) < 0.05, "no lateral twist was commanded"


def test_the_same_seed_gives_the_same_episode() -> None:
    """Seeding has to reach the hardware model too.

    reset() used to rebind self._rng, but the twist and pose estimators hold
    the generator they were constructed with, so their noise carried over
    between episodes and one seed produced different outcomes.
    """
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(6.0, 6.0), n_random=3), seed=0)
    rng = np.random.default_rng(0)

    def rollout(seed: int) -> list[float]:
        obs, _ = env.reset(seed=seed)
        trace = [float(obs.sum())]
        for _ in range(40):
            obs, reward, term, trunc, _info = env.step(
                rng.uniform(-1, 1, 3).astype(np.float32)
            )
            trace.append(float(obs.sum()) + reward)
            if term or trunc:
                break
        return trace

    first = rollout(7)
    rng = np.random.default_rng(0)
    assert rollout(7) == pytest.approx(first)
