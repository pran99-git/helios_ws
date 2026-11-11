# omnivla_server

The OmniVLA policy, wrapped in a small HTTP service. **Not a ROS package** and
not built by colcon; it runs under its own Python.

## Why it is separate

OmniVLA pins `torch==2.2.0` and a git fork of transformers. No such wheel
exists for JetPack 7, and those pins cannot share an interpreter with ROS 2
Jazzy on Python 3.12. See [`../README.md`](../README.md) for what the split
buys beyond avoiding that collision.

The service is **stateless**. The caller owns the observation history, so this
process can be restarted mid-run without resynchronising anything.

## API

```
GET  /health -> {"ready": bool, "context_frames": 6, "chunk_size": 8}

POST /act
  { "frames_b64":     [6 JPEGs, oldest first, current last],
    "goal_text":      str  | null,
    "goal_image_b64": str  | null,
    "goal_pose":      [x, y, yaw] | null }     # metres/radians, body frame
->
  { "waypoints":   [[dx, dy, cos, sin] x 8],   # METRES, cumulative
    "modality_id": int,
    "latency_ms":  float }
```

`dx`/`dy` are **cumulative** displacements from the robot, not per-step deltas:
the model applies `cumsum` internally. Integrating them again squares the
trajectory's reach.

Invalid goal combinations (all three set, or none) return **422** rather than a
guess. `cos`/`sin` arrive unit-normalised and are passed through untouched.

```
POST /score
  { "views_b64": [JPEGs, one per candidate],
    "phrases":   [str, ...] }
->
  { "scores":     [[cosine similarity] x phrases] x views,   # typically 0.15-0.35
    "latency_ms": float }
```

`/score` answers "which of these directions looks like what I was told to
find", for the exploration goal selector. It is a **separate endpoint on
purpose**, and goes through CLIP's own towers rather than the action decoder.

The reason is measured, not stylistic. The decoder's `seq_len` is
`context_size + 2 + 1 + 1 + 1` = 10 tokens, and
`MultiLayerDecoder_mask3.forward` ends in `torch.mean(x, dim=1)`, so a text-only
goal is **one token averaged in with six image tokens**. The result is that the
policy does not ground language: directional words are inert (`"the hallway on
the LEFT"` varies by 1.7 degrees across five scenes), and `"stop"` produces
output statistically identical to a real goal (mean plan length 0.959 m vs
0.969 m). CLIP's image tower, on the same frames, classifies those scenes
correctly at 2.5 ms each.

So language enters through CLIP and the policy drives to coordinates. `/score`
reuses the CLIP instance the policy already loads, so no second copy of the
weights is held, and it uses `clip.load`'s own transform rather than the
vendor's observation transform, which would feed the image tower a
normalisation it was not trained with.

Similarities are comparable **across views for one phrase**, and are **not**
calibrated across phrases. Normalise per phrase before comparing.

## Files

| File | What |
|---|---|
| `vendor.py` | **The only** coupling to the submodule. Three imported symbols |
| `preprocessing.py` | Tensor assembly, goal encoding, constant caching |
| `backends/base.py` | `PolicyBackend` protocol |
| `backends/edge.py` | OmniVLA-edge wrapper, plus a standalone benchmark |
| `server.py` | FastAPI app and entry point |
| `requirements.txt` | Pinned to the exact versions verified on this Jetson |

### `vendor.py`

`utils_policy` uses bare imports, so the submodule's `inference/` directory has
to be on `sys.path` before it is imported. The vendor script gets that for free
because Python adds a script's own directory; we do not.

That ordering is load-bearing. If a formatter ever hoists the `from
utils_policy import ...` line above the `sys.path.insert`, the module breaks.

### `backends/edge.py`

Per-prompt CLIP embeddings are cached. That is 14 ms of a 95 ms inference, and
goals change far less often than frames.

## Run

```bash
cd src/vla_pkg/omnivla_server
env -u PYTHONPATH ../../../.venvs/omnivla_edge/bin/python server.py
```

`unset PYTHONPATH` is not optional. Sourcing ROS prepends `/opt/ros/jazzy` to
it, a virtualenv does **not** override `PYTHONPATH`, and the server would then
import ROS's packages instead of its own. The launch file does this for you.

## Verify

```bash
# benchmark the backend alone, no HTTP
env -u PYTHONPATH ../../../.venvs/omnivla_edge/bin/python -m backends.edge
#   -> inference 95.1 ms -> 10.5 Hz

curl -s localhost:8777/health
```

## Regression fixture

Against the vendor's own sample frame with a language goal, the first and last
waypoints are:

```
[0.12248,  0.00146, 0.99963,  0.02702]
[1.05336, -0.21686, 0.97116, -0.23841]
```

These match the vendor's `run_omnivla_edge.py` output exactly, scaled by
`metric_waypoint_spacing = 0.1`. `test_chunk_decoding.py` pins them. If a
submodule bump changes the numbers, that test is the alarm.

## Adding a backend

Implement `PolicyBackend` and swap it in `server.py:main`. The 7.5B checkpoint
and a TensorRT engine are the intended next two. Nothing in `vla_bridge`
changes, because the HTTP contract does not.

## Related

- [`../README.md`](../README.md)
- [`../vla_bridge/README.md`](../vla_bridge/README.md)
