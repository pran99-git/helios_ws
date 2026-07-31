# Copyright 2026
#
# QPPS (max quadrature pulses/sec) calibration for the A4WD3's four RoboClaw
# motor channels. QPPS is the "top speed" reference the RoboClaw's closed-
# loop velocity PID (SpeedAccel) scales against, so it must be measured per
# motor -- gearbox/motor variance means one shared value would leave 1-2
# wheels under- or over-driven relative to their commanded setpoint.
#
# Reads both RoboClaws directly (NOT through ROS), exactly like
# wheel_monitor.py -- same reason: roboclaw_driver_node is the sole owner of
# the serial ports, so it must be stopped first.
#
# WHEELS MUST BE OFF THE GROUND. This drives one motor at a time up to
# --duty (default 1.0 = 100%) to find its free-spin top speed.
#
# Isolating a single wheel is NOT possible through /cmd_vel: mecanum has
# only 3 controllable DOF (vx, vy, omega) for 4 wheels, so no body-twist
# command can zero three wheels while driving the fourth. This script talks
# to each RoboClaw channel directly instead, holding the untouched
# controller at (0, 0) throughout.
#
# Usage:
#   ros2 run low_level_control_pkg calibrate_qpps
#   ros2 run low_level_control_pkg calibrate_qpps --duty 0.3   # cautious dry run first
#   ros2 run low_level_control_pkg calibrate_qpps --yes        # skip the confirm prompt

import argparse
import sys
import time

from teleop.roboclaw_driver import RoboClaw, SerialBus

# (controller, channel) -- channel 1=M1, 2=M2. Matches the fixed wiring used
# throughout this package: LEFT ctrl (ACM0) front=M2/rear=M1, RIGHT ctrl
# (ACM1) front=M1/rear=M2.
WHEEL_SPEC = {
    'front_left':  ('left', 2),
    'rear_left':   ('left', 1),
    'front_right': ('right', 1),
    'rear_right':  ('right', 2),
}


def parse_args(argv):
    p = argparse.ArgumentParser(description='Per-motor QPPS calibration for '
                                            'the A4WD3 RoboClaws.')
    p.add_argument('--left-port', default='/dev/roboclaw_left')
    p.add_argument('--right-port', default='/dev/roboclaw_right')
    p.add_argument('--left-address', type=lambda x: int(x, 0), default=0x80)
    p.add_argument('--right-address', type=lambda x: int(x, 0), default=0x80)
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--duty', type=float, default=1.0,
                    help='Target duty fraction (0-1) to measure QPPS at. '
                         'Use a low value first as a dry run before 1.0.')
    p.add_argument('--ramp-time', type=float, default=0.5,
                    help='Seconds to ramp 0 -> target duty (and back).')
    p.add_argument('--hold-time', type=float, default=2.0,
                    help='Seconds to hold at target duty before sampling.')
    p.add_argument('--sample-time', type=float, default=1.0,
                    help='Seconds of steady-state samples to average.')
    p.add_argument('--pause-time', type=float, default=2.0,
                    help='Seconds to rest between wheels.')
    p.add_argument('--yes', action='store_true',
                    help='Skip the "wheels are off the ground" confirmation.')
    known, _ = p.parse_known_args(argv)
    return known


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)

    if not args.yes:
        print('!! Wheels MUST be off the ground / rover secured on stands.')
        print(f'!! This will spin each motor individually up to {args.duty*100:.0f}% duty.')
        if input('Type "yes" to continue: ').strip().lower() != 'yes':
            print('Aborted.')
            return 1

    buses = {}

    def get_bus(port):
        if port not in buses:
            buses[port] = SerialBus(port, args.baud, timeout=0.1)
        return buses[port]

    try:
        left = RoboClaw(get_bus(args.left_port), args.left_address)
        right = RoboClaw(get_bus(args.right_port), args.right_address)
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR opening serial port: {exc}', file=sys.stderr)
        print('Is roboclaw_driver_node still running? Stop it first '
              '(a serial port can only be opened once).', file=sys.stderr)
        return 1

    controllers = {'left': left, 'right': right}

    def set_duty(name, m1, m2):
        ok = controllers[name].duty_m1m2(m1, m2)
        if not ok:
            print(f'  WARNING: {name} duty command not acknowledged', file=sys.stderr)

    def all_stop():
        set_duty('left', 0.0, 0.0)
        set_duty('right', 0.0, 0.0)

    results = {}
    try:
        all_stop()
        time.sleep(0.2)

        for wheel, (ctrl_name, channel) in WHEEL_SPEC.items():
            other_ctrl = 'right' if ctrl_name == 'left' else 'left'
            print(f'\n--- {wheel} ({ctrl_name} ctrl, M{channel}) ---')
            set_duty(other_ctrl, 0.0, 0.0)

            # Ramp up.
            steps = max(1, int(args.ramp_time * 20))
            for i in range(1, steps + 1):
                d = args.duty * i / steps
                m1, m2 = (d, 0.0) if channel == 1 else (0.0, d)
                set_duty(ctrl_name, m1, m2)
                time.sleep(args.ramp_time / steps)

            print(f'  holding at {args.duty*100:.0f}% duty for {args.hold_time}s...')
            time.sleep(args.hold_time)

            # Sample steady-state speed.
            samples = []
            t_end = time.time() + args.sample_time
            while time.time() < t_end:
                speeds = controllers[ctrl_name].read_speeds()
                if speeds is not None:
                    samples.append(speeds[channel - 1])
                time.sleep(0.05)

            # Ramp down.
            for i in range(steps, -1, -1):
                d = args.duty * i / steps
                m1, m2 = (d, 0.0) if channel == 1 else (0.0, d)
                set_duty(ctrl_name, m1, m2)
                time.sleep(args.ramp_time / steps)

            if samples:
                qpps = sum(samples) / len(samples)
                results[wheel] = qpps
                print(f'  {wheel}: QPPS = {qpps:.0f} counts/sec '
                      f'({len(samples)} samples, min={min(samples):.0f}, max={max(samples):.0f})')
            else:
                print(f'  {wheel}: NO VALID SPEED READS -- check wiring/serial', file=sys.stderr)

            time.sleep(args.pause_time)

    except KeyboardInterrupt:
        print('\ninterrupted.')
    finally:
        all_stop()
        for bus in buses.values():
            bus.close()

    if results:
        print('\n=== Summary (paste into teleop.yaml) ===')
        for wheel in WHEEL_SPEC:
            if wheel in results:
                print(f'# {wheel}_qpps: {results[wheel]:.0f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
