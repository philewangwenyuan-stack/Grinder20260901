#!/usr/bin/env python3
"""Minimal RS485 motor command-to-feedback latency test.

This intentionally bypasses the ROS chassis driver. Stop the normal
``chassis_driver`` first so that it does not share the RS485 port or add
unrelated Modbus traffic while this test is running.
"""

import argparse
import datetime
import math
import os
import sys
import time

# Keep direct execution useful outside a sourced catkin workspace.
_PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if _PACKAGE_SRC not in sys.path:
    sys.path.insert(0, _PACKAGE_SRC)

from grinder_chassis_driver.register_map import (
    ACTUAL_SPEED_READ_COUNT,
    INT16_MAX,
    INT16_MIN,
    REGISTER_LEFT_MOTOR_ACTUAL_SPEED,
    REGISTER_LEFT_WHEEL_SPEED,
    clamp,
)


def wall_time():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def sign(value):
    return -1 if int(value) < 0 else 1


def map_wheel_command(left, right, left_sign, right_sign, swap_lr):
    """Match ChassisDriverNode._apply_drive_direction()."""
    if swap_lr:
        left, right = right, left
    return int(left) * sign(left_sign), int(right) * sign(right_sign)


def map_feedback_speed(left, right, left_sign, right_sign, swap_lr):
    """Match motor_rpm_to_base_twist() feedback sign/order correction."""
    left = int(left) * sign(left_sign)
    right = int(right) * sign(right_sign)
    if swap_lr:
        left, right = right, left
    return left, right


def cmd_vel_to_wheel_rpm(linear_mps, angular_radps, args):
    """Match the driver's /cmd_vel differential-drive conversion."""
    if args.wheel_radius_m <= 0.0 or args.wheel_track_m <= 0.0 or args.gear_ratio <= 0.0:
        raise ValueError("wheel-radius-m, wheel-track-m, and gear-ratio must be greater than zero")
    left_mps = float(linear_mps) - float(angular_radps) * (args.wheel_track_m * 0.5)
    right_mps = float(linear_mps) + float(angular_radps) * (args.wheel_track_m * 0.5)
    rpm_factor = 60.0 / (2.0 * math.pi * args.wheel_radius_m)
    left_rpm = left_mps * rpm_factor * args.gear_ratio * args.cmd_vel_scale
    right_rpm = right_mps * rpm_factor * args.gear_ratio * args.cmd_vel_scale
    return round(left_rpm), round(right_rpm)


def response_reached(target, baseline, actual, threshold_rpm, zero_tolerance_rpm):
    """Return whether one motor has visibly reacted to this command."""
    if int(target) == 0:
        return abs(int(actual)) <= max(0, int(zero_tolerance_rpm))
    return abs(int(actual) - int(baseline)) >= max(1, int(threshold_rpm))


def direction_text(target, actual):
    if int(target) == 0 or int(actual) == 0:
        return "n/a"
    return "OK" if int(target) * int(actual) > 0 else "WRONG"


def read_actual_speeds(transport):
    return transport.read_register_block(
        REGISTER_LEFT_MOTOR_ACTUAL_SPEED,
        ACTUAL_SPEED_READ_COUNT,
        signed_indices=(0, 1),
    )


def build_parser():
    parser = argparse.ArgumentParser(
        description="Send one wheel command and measure Modbus ACK plus motor feedback latency."
    )
    parser.add_argument("--port", default="/dev/ttyS0")
    parser.add_argument("--baudrate", type=int, default=19200)
    parser.add_argument("--slave-id", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=0.1)

    parser.add_argument("--left-rpm", type=int, default=100, help="logical left wheel command")
    parser.add_argument("--right-rpm", type=int, default=100, help="logical right wheel command")
    parser.add_argument(
        "--linear-mps",
        type=float,
        default=None,
        help="optional /cmd_vel-style linear.x; when set, wheel RPM arguments are ignored",
    )
    parser.add_argument("--angular-radps", type=float, default=0.0)
    parser.add_argument("--wheel-track-m", type=float, default=0.70)
    parser.add_argument("--wheel-radius-m", type=float, default=0.1475)
    parser.add_argument("--gear-ratio", type=float, default=60.0)
    parser.add_argument("--cmd-vel-scale", type=float, default=1.0)

    # Defaults match the current chassis_driver.yaml direction parameters.
    parser.add_argument("--drive-left-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--drive-right-sign", type=int, choices=(-1, 1), default=-1)
    parser.add_argument("--drive-swap-lr", action="store_true")
    parser.add_argument("--cmd-vel-drive-left-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--cmd-vel-drive-right-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--cmd-vel-drive-swap-lr", action="store_true")
    parser.add_argument("--actual-speed-left-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--actual-speed-right-sign", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--actual-speed-swap-lr", action="store_true")

    parser.add_argument("--threshold-rpm", type=int, default=10)
    parser.add_argument("--zero-tolerance-rpm", type=int, default=10)
    parser.add_argument("--wait-timeout", type=float, default=3.0)
    parser.add_argument("--poll-hz", type=float, default=5.0)
    parser.add_argument(
        "--hold-after-response",
        type=float,
        default=0.0,
        help="keep the command active this many seconds after both motors respond",
    )
    parser.add_argument(
        "--raw-log-file",
        default="",
        help="optional raw frame log; disabled by default to keep this test quiet",
    )
    return parser


def run(args):
    # modbus_transport uses termios, which is available on the target Linux
    # controller but not on Windows development machines. Keep --help and the
    # pure conversion helpers usable on either platform.
    try:
        from grinder_chassis_driver.modbus_transport import ModbusTransport, ModbusTransportError
    except ImportError as exc:
        print("[{}] ERROR RS485 transport is unavailable: {}".format(wall_time(), exc), file=sys.stderr, flush=True)
        return 1

    if args.linear_mps is None:
        logical_left = int(args.left_rpm)
        logical_right = int(args.right_rpm)
        raw_left, raw_right = map_wheel_command(
            logical_left,
            logical_right,
            args.drive_left_sign,
            args.drive_right_sign,
            args.drive_swap_lr,
        )
        source_text = "wheel_speed_cmd"
    else:
        logical_left, logical_right = cmd_vel_to_wheel_rpm(args.linear_mps, args.angular_radps, args)
        raw_left, raw_right = map_wheel_command(
            logical_left,
            logical_right,
            args.cmd_vel_drive_left_sign,
            args.cmd_vel_drive_right_sign,
            args.cmd_vel_drive_swap_lr,
        )
        source_text = "cmd_vel conversion"

    raw_left = int(clamp(raw_left, INT16_MIN, INT16_MAX))
    raw_right = int(clamp(raw_right, INT16_MIN, INT16_MAX))
    transport = ModbusTransport(
        port=args.port,
        slave_id=args.slave_id,
        baudrate=args.baudrate,
        timeout=args.timeout,
        raw_log_enabled=bool(args.raw_log_file),
        raw_log_file=args.raw_log_file,
    )
    command_sent = False
    response_times = {}

    print("[{}] TEST start source={}".format(wall_time(), source_text), flush=True)
    print(
        "[{}] command logical=({},{}) rpm -> register=({},{}) rpm".format(
            wall_time(), logical_left, logical_right, raw_left, raw_right
        ),
        flush=True,
    )
    print(
        "[{}] port={} baudrate={} slave={} (stop chassis_driver before test)".format(
            wall_time(), args.port, args.baudrate, args.slave_id
        ),
        flush=True,
    )

    try:
        transport.connect()
        baseline = read_actual_speeds(transport)
        print(
            "[{}] baseline feedback=({},{}) rpm".format(
                wall_time(), baseline[0], baseline[1]
            ),
            flush=True,
        )

        command_started = time.monotonic()
        print(
            "[{}] SEND register 0x{:04X}: left={} right={} rpm".format(
                wall_time(), REGISTER_LEFT_WHEEL_SPEED, raw_left, raw_right
            ),
            flush=True,
        )
        transport.write_register_block(REGISTER_LEFT_WHEEL_SPEED, [raw_left, raw_right])
        command_sent = True
        ack_ms = (time.monotonic() - command_started) * 1000.0
        print(
            "[{}] WRITE_ACK elapsed={:.3f} ms".format(wall_time(), ack_ms),
            flush=True,
        )

        deadline = time.monotonic() + max(0.0, args.wait_timeout)
        sample_period = 1.0 / max(0.1, args.poll_hz)
        next_sample = time.monotonic()
        while time.monotonic() <= deadline:
            delay = next_sample - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            sample_started = time.monotonic()
            actual = read_actual_speeds(transport)
            logical_feedback_left, logical_feedback_right = map_feedback_speed(
                actual[0],
                actual[1],
                args.actual_speed_left_sign,
                args.actual_speed_right_sign,
                args.actual_speed_swap_lr,
            )
            sample_elapsed_ms = (time.monotonic() - sample_started) * 1000.0
            elapsed_ms = (time.monotonic() - command_started) * 1000.0
            print(
                "[{}] FEEDBACK elapsed={:.3f} ms read={:.3f} ms raw=({:>6},{:>6}) logical=({:>6},{:>6}) rpm dir=({},{})".format(
                    wall_time(),
                    elapsed_ms,
                    sample_elapsed_ms,
                    actual[0],
                    actual[1],
                    logical_feedback_left,
                    logical_feedback_right,
                    direction_text(logical_left, logical_feedback_left),
                    direction_text(logical_right, logical_feedback_right),
                ),
                flush=True,
            )

            for name, target, before, value in (
                ("left", raw_left, baseline[0], actual[0]),
                ("right", raw_right, baseline[1], actual[1]),
            ):
                if name in response_times:
                    continue
                if response_reached(
                    target,
                    before,
                    value,
                    args.threshold_rpm,
                    args.zero_tolerance_rpm,
                ):
                    response_times[name] = (time.monotonic() - command_started) * 1000.0
                    print(
                        "[{}] MOTOR_RESPONSE motor={} elapsed={:.3f} ms feedback={} rpm direction={}".format(
                            wall_time(),
                            name,
                            response_times[name],
                            value,
                            direction_text(
                                logical_left if name == "left" else logical_right,
                                logical_feedback_left if name == "left" else logical_feedback_right,
                            ),
                        ),
                        flush=True,
                    )

            if len(response_times) == 2:
                break
            next_sample += sample_period

        missing = [name for name in ("left", "right") if name not in response_times]
        if missing:
            print(
                "[{}] RESULT timeout waiting for {} motor response(s)".format(
                    wall_time(), ",".join(missing)
                ),
                flush=True,
            )
            return 2
        print(
            "[{}] RESULT ACK={:.3f} ms left_response={:.3f} ms right_response={:.3f} ms".format(
                wall_time(), ack_ms, response_times["left"], response_times["right"]
            ),
            flush=True,
        )
        if args.hold_after_response > 0.0:
            print(
                "[{}] HOLD command active for {:.3f} s before safety stop".format(
                    wall_time(), args.hold_after_response
                ),
                flush=True,
            )
            time.sleep(args.hold_after_response)
        return 0
    except (ModbusTransportError, OSError, ValueError) as exc:
        print("[{}] ERROR {}".format(wall_time(), exc), file=sys.stderr, flush=True)
        return 1
    finally:
        if command_sent:
            try:
                stop_started = time.monotonic()
                transport.write_register_block(REGISTER_LEFT_WHEEL_SPEED, [0, 0])
                print(
                    "[{}] SAFETY_STOP_ACK elapsed={:.3f} ms".format(
                        wall_time(), (time.monotonic() - stop_started) * 1000.0
                    ),
                    flush=True,
                )
            except Exception as exc:
                print("[{}] SAFETY_STOP_ERROR {}".format(wall_time(), exc), file=sys.stderr, flush=True)
        transport.close()


def main():
    return run(build_parser().parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
