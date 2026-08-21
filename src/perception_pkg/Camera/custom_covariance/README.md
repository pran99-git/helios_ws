# custom_covariance

Workspace-owned corrections to sensor uncertainty that the vendor drivers do not
provide.

It sits next to `zed-ros2-wrapper` so that everything camera-related lives in
`Camera/`, but **it is our code, not the vendor's**. `zed-ros2-wrapper` is a git
submodule; anything written inside it is reverted by `git submodule update`.
This package is outside that boundary, so it survives.

Currently one node: `zed_odom_covariance_node`.

---

## Why it exists

The ZED wrapper publishes `/zed/zed_node/odom` with **`twist.covariance` set to
all zeros**. That is not a misconfiguration and no parameter changes it. In
`zed_camera_component_main.cpp`, `ZedCamera::publishOdom()`:

```cpp
auto odomMsg = std::make_unique<nav_msgs::msg::Odometry>();   // all covariances zero
...
odomMsg->pose.covariance[i] = slPose.pose_covariance[i];      // pose IS filled
...
odomMsg->twist.twist.linear.x = linear_velocity.x();          // twist VALUES filled
...                                                            // twist.covariance never assigned
mPubOdom->publish(std::move(odomMsg));
```

The root cause is upstream of the wrapper: the SDK's `sl::Pose` exposes a pose
covariance but has no velocity covariance to copy from.

That matters because `sensor_fusion/config/ekf.yaml` fuses **only the twist**
half of this message (`vx`, `vy`, `vyaw`). Measured on this rover using
`robot_localization`'s own `debug_out_file`:

| | raw wrapper topic |
|---|---|
| `very small error covariance` warnings | **34044** in ~30 s |
| substituted variance | **1e-9** |
| Kalman gain for `vx`/`vy`/`vyaw` | **exactly 1.0** |
| twist measurements rejected by the Mahalanobis gate | **5867 / 29003 (~20%)** |

A gain of 1.0 means the corrected state equalled the measurement digit for
digit: the camera **overwrote** the wheel odometry instead of being blended with
it, and the covariance tuning in `wheel_odometry.yaml` was bypassed entirely.
Meanwhile the same 1e-9 made ordinary innovations look enormous to the rejection
gate, so a fifth of the data was thrown away. Over-trusted when accepted,
discarded otherwise — neither is fusion.

The warnings only reach `robot_localization`'s debug stream, never the console,
which is why this went unnoticed.

**Why it cannot be fixed in `ekf.yaml`:** `robot_localization` has no per-input
covariance override. Dumping every parameter the node accepts gives exactly
three covariance parameters — `process_noise_covariance`,
`initial_estimate_covariance`, `dynamic_process_noise_covariance` — all
filter-level, none per-sensor. Measurement covariance can only arrive *on the
message*. Adding something like `odom1_twist_covariance:` to `ekf.yaml` is
silently ignored: no error, no warning, no effect.

---

## What the node does

```
/zed/zed_node/odom  ──▶  zed_odom_covariance_node  ──▶  /zed/odom_with_cov  ──▶  ekf.yaml odom1
   twist.covariance                                        twist.covariance
   all zeros                                               from config/
```

Header, `child_frame_id`, pose, pose covariance and the twist *values* pass
through untouched. Only `twist.covariance` is replaced.

The **pose** covariance is deliberately left alone — the SDK populates it with
real, live-varying data, and `ekf.yaml` does not fuse the pose anyway.

This is the same thing `wheel_odometry_node` already does for its own output.
The wheels get their covariance from a YAML we own; this gives the camera the
same treatment, because its vendor does not.

### Two safeguards

- **Rejects a zero or wrong-length diagonal at startup** rather than starting
  and silently recreating the original bug.
- **Warns once if the incoming `twist.covariance` is non-zero**, which would
  mean Stereolabs started providing it and our hardcoded numbers are now
  overriding real SDK data. Delete this node at that point.

---

## Running it

`sensor_fusion/launch/bringup.launch.py` starts it automatically alongside the
camera, gated on the same `camera:=` argument. Nothing extra to launch.

Standalone:

```bash
ros2 run custom_covariance zed_odom_covariance_node --ros-args \
    --params-file $(ros2 pkg prefix --share custom_covariance)/config/zed_odom_covariance.yaml
```

Verify it is working:

```bash
ros2 topic echo /zed/zed_node/odom  --field twist.covariance   # all zeros
ros2 topic echo /zed/odom_with_cov  --field twist.covariance   # diagonal set
```

---

## Tuning

Values live in `config/zed_odom_covariance.yaml`, laid out to be read side by
side with `wheel_odometry.yaml` — because the **ratio between the two files is
the entire fusion policy**. The EKF has no other mechanism for deciding which
sensor to believe.

| axis | wheel | ZED | who wins, and why |
|---|---|---|---|
| `vx` | 0.01 | 0.02 | Wheels — encoders are direct in straight-line rolling |
| `vy` | 0.05 | 0.02 | ZED — mecanum rollers make lateral wheel data untrustworthy; VIO sees the body actually move |
| `vyaw` | 0.05 | 0.01 | ZED — its odom is visual-*inertial*, and a gyro beats a rate differenced from wheel speeds |

> **Do not derive these from a stationary recording.** Parked, this camera
> reports ~2e-5 m/s of noise; a variance computed from that is ~1e-9 — exactly
> the number that caused the original bug. Refine from a moving run instead.

---

## Verified

Measured with the real camera, `robot_localization` debug output, before and
after:

| | before | after |
|---|---|---|
| `very small error covariance` warnings | 34044 | **0** |
| Kalman gain `vx` / `vy` / `vyaw` | 1.0 / 1.0 / 1.0 | **0.169 / 0.169 / 0.197** |
| corrected state vs measurement | identical | **differs — actually blending** |
| Mahalanobis rejection rate | 5867/29003 (20%) | **14/2407 (0.6%)** |
| measurement covariance in use | 1e-9 | **0.02 / 0.02 / 0.01** |

---

## Related

- `sensor_fusion/config/ekf.yaml` — `odom1` must point at `/zed/odom_with_cov`,
  never the raw wrapper topic.
- `wheel_odometry/config/wheel_odometry.yaml` — the other half of the ratio.
- `sensor_fusion/launch/bringup.launch.py` — starts this node with the camera.

`odom1_twist_rejection_threshold` in `ekf.yaml` still **needs re-tuning**. It was
keyed off the substituted 1e-9; with a real covariance the innovation distances
collapse by ~4000x, so the current value of 2.0 is close to inert and will not
catch a genuine VIO failure. Re-measure on a moving run.
