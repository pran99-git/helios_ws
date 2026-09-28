"""Synthetic telemetry fixture. No ROS imports and no hardware connections."""
import math
import threading
import time
from pathlib import Path

from .operations import OperationManager
from .processes import SimulatedProcessBackend

from .server import TelemetryServer, arguments, token_from_environment
from .state import TelemetryState


def main():
    args, extra = arguments("SYNTHETIC FIXTURE ONLY: no ROS or robot connection")
    if extra:
        raise ValueError("Unknown fixture arguments")
    state = TelemetryState(source="fixture")
    stop = threading.Event()
    started = time.monotonic()

    def update():
        while not stop.is_set():
            phase = (time.monotonic() - started) * 0.1
            state.odometry(frame_id="odom", child_frame_id="base_link",
                           x=math.sin(phase), y=1 - math.cos(phase),
                           qx=0.0, qy=0.0, qz=math.sin(phase / 2), qw=math.cos(phase / 2),
                           linear_x=0.1, linear_y=0.0, angular_z=0.1)
            state.scan(frame_id="laser", ranges=[1.2 + 0.1 * math.sin(phase), 2.0],
                       range_min=0.02, range_max=10.0)
            stop.wait(0.1)

    operations = OperationManager(SimulatedProcessBackend(), Path(args.workspace), enabled=True, simulated=True)
    server = TelemetryServer((args.host, args.port), state, token_from_environment(), operations=operations)
    updater = threading.Thread(target=update, daemon=True)
    updater.start()
    print(f"SYNTHETIC FIXTURE ONLY at {args.host}:{server.server_port}; no ROS or hardware", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        operations.shutdown()
        stop.set()
        updater.join(timeout=2)
        server.server_close()


if __name__ == "__main__":
    main()
