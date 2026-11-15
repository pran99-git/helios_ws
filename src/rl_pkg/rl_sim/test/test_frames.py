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


def test_slides_are_declared_before_the_hinge(model: mujoco.MjModel) -> None:
    """Joint ORDER decides which frame the slides live in.

    Hinge first expresses them in the rotating frame, which makes world
    position Rz(yaw) applied to the slide coordinates and orbits the body
    around the origin whenever it turns.
    """
    names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        for j in range(model.njnt)
    ]
    assert names.index("slide_x") < names.index("hinge_z")
    assert names.index("slide_y") < names.index("hinge_z")


def test_model_actuators_are_world_axes(model: mujoco.MjModel) -> None:
    """At the MODEL level the slides are world axes, whatever the heading.

    Turning a body twist into world axes is rover_env's job, not the
    model's; test_forward_follows_the_nose_away_from_the_origin covers that
    half.
    """
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    adr = addresses(model)
    for yaw_deg in (0.0, 90.0, -135.0):
        data = mujoco.MjData(model)
        data.qpos[adr["hinge_z"]] = np.deg2rad(yaw_deg)
        mujoco.mj_forward(model, data)
        start = data.xpos[body][:2].copy()
        data.ctrl[:] = [0.40, 0.0, 0.0]
        for _ in range(400):
            mujoco.mj_step(model, data)
        moved = data.xpos[body][:2] - start
        assert moved[0] > 0.3, "the x actuator must drive world +x"
        assert abs(moved[1]) < 0.05


def test_env_reports_the_composed_world_transform() -> None:
    """`_true_pose` reads xpos, which is right whatever the joint ordering.

    The slide qpos happen to agree with it under world-frame slides, but
    only up to integration lag, and they stop agreeing entirely if the
    joint order ever changes again.
    """
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(8.0, 8.0)), seed=0)
    env.reset(seed=0)
    body = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    for _ in range(50):
        env.step(np.array([1.0, 0.0, 0.3], dtype=np.float32))

    assert env._true_pose()[:2] == pytest.approx(env.data.xpos[body][:2], abs=1e-9)


def test_start_pose_lands_where_it_was_asked_to() -> None:
    """The rover must begin the episode on the route the planner built."""
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(8.0, 8.0)), seed=1)
    for seed in range(20):
        env.reset(seed=seed)
        wanted = env.route[0]
        got = env._true_pose()
        assert got[0] == pytest.approx(wanted.x, abs=1e-6)
        assert got[1] == pytest.approx(wanted.y, abs=1e-6)


def test_measured_twist_is_body_frame() -> None:
    """The slides are world axes, so the reported twist is rotated back.

    The observation and /cmd_vel are both body frame; reporting world
    velocity would make the policy's own last command unrecognisable to it.
    """
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


def _place(env: RoverEnv, x: float, y: float, yaw_deg: float) -> None:
    """Put the rover somewhere specific, with no command history."""
    adr = addresses(env.model)
    env.data.qpos[:] = 0.0
    env.data.qvel[:] = 0.0
    env.data.qpos[adr["slide_x"]] = x
    env.data.qpos[adr["slide_y"]] = y
    env.data.qpos[adr["hinge_z"]] = np.deg2rad(yaw_deg)
    mujoco.mj_forward(env.model, env.data)
    env._rate.reset()
    env._delay.reset()


@pytest.mark.parametrize(("x", "y"), [(0.0, 0.0), (2.0, 0.0), (3.0, 1.0), (-2.5, 2.0)])
def test_rotating_does_not_move_the_rover(x: float, y: float) -> None:
    """The bug every other frame test missed by starting at the origin.

    With the hinge ahead of the slides, a pure rotation 2 m out moved the
    body 2.54 m. The policy then learned never to turn, which was correct:
    turning flung it across the arena into obstacles.
    """
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(20.0, 20.0)), seed=0)
    env.reset(seed=0)
    _place(env, x, y, 0.0)
    before = env._true_pose().copy()
    for _ in range(40):
        env.step(np.array([0.0, 0.0, 1.0], dtype=np.float32))
    after = env._true_pose()
    assert abs(after[2] - before[2]) > 0.5, "it should actually have turned"
    assert np.linalg.norm(after[:2] - before[:2]) < 0.01


@pytest.mark.parametrize(
    ("yaw_deg", "axis", "sign"),
    [(0.0, 0, +1), (90.0, 1, +1), (180.0, 0, -1), (-90.0, 1, -1)],
)
def test_forward_follows_the_nose_away_from_the_origin(
    yaw_deg: float, axis: int, sign: int
) -> None:
    """Same claim as the model-level test, but through the env and off-origin.

    Catches the other half: world-frame slides need the command rotated into
    world axes, and actuator limits wide enough to realise it. Body limits on
    world axes clipped full forward at yaw 90 to the lateral ceiling.
    """
    env = RoverEnv(scenario=scenarios.Scenario(bounds=(20.0, 20.0)), seed=0)
    env.reset(seed=0)
    _place(env, 2.0, -1.0, yaw_deg)
    before = env._true_pose().copy()
    for _ in range(40):
        env.step(np.array([1.0, 0.0, 0.0], dtype=np.float32))
    moved = env._true_pose()[:2] - before[:2]
    assert sign * moved[axis] > 0.5
    assert abs(moved[1 - axis]) < 0.05
    assert env._true_twist()[0] > 0.35, "body vx must reach the real limit"
