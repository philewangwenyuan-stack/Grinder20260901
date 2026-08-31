import math
from dataclasses import dataclass


INT16_MIN = -32768
INT16_MAX = 32767

REGISTER_LEFT_WHEEL_SPEED = 0x4010
REGISTER_RIGHT_WHEEL_SPEED = 0x4011
REGISTER_DISC_SPEED = 0x4012
REGISTER_DISC_ENABLE = 0x4013
REGISTER_WORK_MODE = 0x4014
REGISTER_DISC_LIFT = 0x4015
REGISTER_LIGHT = 0x4016
REGISTER_LEFT_MOTOR_ACTUAL_SPEED = 0x4017
REGISTER_RIGHT_MOTOR_ACTUAL_SPEED = 0x4018
ACTUAL_SPEED_READ_COUNT = 2

READ_BLOCK_START = REGISTER_LEFT_WHEEL_SPEED
READ_BLOCK_COUNT = 7

DISC_ENABLE_OFF = 0x0000
DISC_ENABLE_ON = 0x0001

WORK_MODE_AUTO = 0x0001
WORK_MODE_MANUAL = 0x0002

DISC_LIFT_DOWN = 0x0001
DISC_LIFT_UP = 0x0002

LIGHT_OFF = 0x0000
LIGHT_ON = 0x0001


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def to_uint16(value):
    return value & 0xFFFF


def to_int16(value):
    value &= 0xFFFF
    return value - 0x10000 if value & 0x8000 else value


def motor_rpm_to_base_twist(
    left_motor_rpm,
    right_motor_rpm,
    wheel_radius_m,
    wheel_track_m,
    gear_ratio,
    left_sign=1,
    right_sign=1,
    swap_lr=False,
    motor_speed_scale=1.0,
):
    """Convert signed motor rpm feedback to differential-drive v and omega.

    The feedback registers are assumed to report motor-side rpm.  The wheel
    rpm is therefore motor rpm divided by the reduction ratio.  Direction
    signs are applied to the feedback values so the result follows ROS's
    convention: positive x means forward and positive z means left turn.
    """
    radius = float(wheel_radius_m)
    track = float(wheel_track_m)
    ratio = float(gear_ratio)
    scale = float(motor_speed_scale)
    if radius <= 0.0:
        raise ValueError("wheel_radius_m must be greater than zero")
    if track <= 0.0:
        raise ValueError("wheel_track_m must be greater than zero")
    if ratio <= 0.0:
        raise ValueError("gear_ratio must be greater than zero")
    if scale <= 0.0:
        raise ValueError("motor_speed_scale must be greater than zero")

    left = float(left_motor_rpm) * scale * float(left_sign)
    right = float(right_motor_rpm) * scale * float(right_sign)
    if swap_lr:
        left, right = right, left

    wheel_rpm_to_mps = (2.0 * math.pi * radius) / (60.0 * ratio)
    left_mps = left * wheel_rpm_to_mps
    right_mps = right * wheel_rpm_to_mps
    linear_mps = 0.5 * (left_mps + right_mps)
    angular_radps = (right_mps - left_mps) / track
    return linear_mps, angular_radps


@dataclass
class ChassisCommand:
    left_wheel_speed: int = 0
    right_wheel_speed: int = 0
    disc_speed: int = 0
    disc_enable: int = DISC_ENABLE_OFF
    work_mode: int = WORK_MODE_MANUAL
    disc_lift: int = DISC_LIFT_UP
    light: int = LIGHT_OFF

    def as_register_block(self):
        return [
            to_uint16(self.left_wheel_speed),
            to_uint16(self.right_wheel_speed),
            to_uint16(self.disc_speed),
            self.disc_enable,
            self.work_mode,
            self.disc_lift,
            self.light,
        ]


@dataclass
class RegisterSnapshot:
    left_wheel_speed: int = 0
    right_wheel_speed: int = 0
    disc_speed: int = 0
    disc_enable: int = DISC_ENABLE_OFF
    work_mode: int = WORK_MODE_MANUAL
    disc_lift: int = DISC_LIFT_UP
    light: int = LIGHT_OFF

    @classmethod
    def from_registers(cls, registers):
        if len(registers) < READ_BLOCK_COUNT:
            raise ValueError("Expected at least 7 registers in the snapshot")
        return cls(
            left_wheel_speed=to_int16(registers[0]),
            right_wheel_speed=to_int16(registers[1]),
            disc_speed=to_int16(registers[2]),
            disc_enable=registers[3],
            work_mode=registers[4],
            disc_lift=registers[5],
            light=registers[6],
        )
