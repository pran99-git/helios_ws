"""Thread-safe, monotonic receipt-age telemetry with finite JSON values."""
import math
import threading
import time


class TelemetryState:
    def __init__(self, clock=time.monotonic, stale_after=2.0, source="ros2"):
        if source not in ("ros2", "fixture"):
            raise ValueError("Unknown telemetry source")
        self._source = source
        self._clock = clock
        self._stale_after = stale_after
        self._lock = threading.Lock()
        self._samples = {}

    def _store(self, key, values):
        with self._lock:
            self._samples[key] = (self._clock(), values)

    def odometry(self, *, frame_id, child_frame_id, x, y, qx, qy, qz, qw,
                 linear_x, linear_y, angular_z):
        values = (x, y, qx, qy, qz, qw, linear_x, linear_y, angular_z)
        if not all(math.isfinite(value) for value in values):
            self._store("odometry", None)
            return
        # Scale before normalizing to avoid overflow for large finite inputs.
        scale = max(abs(v) for v in (qx, qy, qz, qw))
        if scale == 0:
            self._store("odometry", None)
            return
        scaled = [v / scale for v in (qx, qy, qz, qw)]
        norm = math.sqrt(sum(v * v for v in scaled))
        qx, qy, qz, qw = (v / norm for v in scaled)
        heading = math.atan2(2 * (qw * qz + qx * qy),
                             1 - 2 * (qy * qy + qz * qz))
        self._store("odometry", dict(frame_id=frame_id, child_frame_id=child_frame_id,
                                    x=x, y=y, heading=heading, linear_x=linear_x,
                                    linear_y=linear_y, angular_z=angular_z))

    def scan(self, *, frame_id, ranges, range_min, range_max):
        if not all(math.isfinite(v) for v in (range_min, range_max)) or range_min < 0 or range_min > range_max:
            self._store("scan", None)
            return
        valid = [v for v in ranges if math.isfinite(v) and range_min <= v <= range_max]
        self._store("scan", dict(frame_id=frame_id, nearest_m=min(valid) if valid else None))

    def snapshot(self):
        with self._lock:
            now = self._clock()
            result = {"api_version": 1, "source": self._source}
            for key in ("odometry", "scan"):
                sample = self._samples.get(key)
                if sample is None:
                    result[key] = {"available": False}
                    continue
                received, values = sample
                age = max(0.0, now - received)
                if values is None or age > self._stale_after:
                    result[key] = {"available": False, "age_seconds": age}
                else:
                    result[key] = {"available": True, "age_seconds": age, **values}
            return result
