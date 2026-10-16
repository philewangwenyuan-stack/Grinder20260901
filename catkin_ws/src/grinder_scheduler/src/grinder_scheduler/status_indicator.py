"""ROS-free indicator state policy and USB protocol (Python 3.8 compatible)."""
import binascii
import math
import secrets
from collections import deque
from enum import IntEnum


class LightState(IntEnum):
    STARTUP = 0
    READY = 1
    AUTOMATIC = 2
    MANUAL = 3
    PAUSED = 4
    COMPLETED = 5
    FAULT = 6
    EMERGENCY = 7
    COLLISION_PAUSED = 8


class StateResolver:
    REQUIRED = ("scheduler", "chassis", "activity", "scheduler_emergency", "chassis_emergency")

    def __init__(self, timeout=3.0, ignore_chassis_status=False):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("state timeout must be positive")
        self.timeout = timeout
        self.ignore_chassis_status = ignore_chassis_status
        self.samples = {}
        self.ever_ready = False

    def update(self, name, value, now):
        self.samples[name] = (value, now)

    def fresh(self, name, now):
        sample = self.samples.get(name)
        return sample is not None and 0 <= now - sample[1] <= self.timeout

    def resolve(self, now):
        # Even stale TRUE emergency records override a late RUNNING status.
        for name in ("scheduler_emergency", "chassis_emergency"):
            if self.samples.get(name, (False, 0))[0]:
                return LightState.EMERGENCY
        scheduler = self.samples.get("scheduler", ({}, 0))[0]
        chassis = {} if self.ignore_chassis_status else self.samples.get("chassis", ({}, 0))[0]
        if (self.fresh("scheduler", now) and scheduler.get("state") == "ERROR") or (
            not self.ignore_chassis_status and self.fresh("chassis", now) and not chassis.get("connected", False)
        ):
            return LightState.FAULT
        required = (name for name in self.REQUIRED
                    if not self.ignore_chassis_status or name not in ("chassis", "chassis_emergency"))
        if not all(self.fresh(name, now) for name in required):
            return LightState.FAULT if self.ever_ready else LightState.STARTUP
        activity = self.samples["activity"][0]
        state = scheduler.get("state")
        if state not in ("IDLE", "READY", "PLANNING", "RUNNING", "PAUSED", "COMPLETED", "STOPPED"):
            return LightState.FAULT
        self.ever_ready = True
        if state == "RUNNING" and activity.get("automatic_active"):
            return LightState.AUTOMATIC
        if state == "PAUSED" and activity.get("pause_reason") == "collision":
            return LightState.COLLISION_PAUSED
        if activity.get("manual_active"):
            return LightState.MANUAL
        if state == "RUNNING":
            return LightState.FAULT  # inconsistent execution state
        if state == "PAUSED":
            return LightState.PAUSED
        if chassis.get("disc_enabled"):
            return LightState.MANUAL
        if state == "COMPLETED":
            return LightState.COMPLETED
        return LightState.READY


def envelope(payload):
    encoded = payload.encode("ascii")
    return encoded + b"*%04X\n" % binascii.crc_hqx(encoded, 0xFFFF)


def unpack(frame):
    if len(frame) > 128 or not frame.endswith(b"\n"):
        raise ValueError("invalid frame length/termination")
    payload, crc = frame[:-1].rsplit(b"*", 1)
    if len(crc) != 4 or int(crc, 16) != binascii.crc_hqx(payload, 0xFFFF):
        raise ValueError("invalid CRC")
    return payload.decode("ascii").split(",")


def state_frame(session, sequence, state, brightness=102, clear=False):
    if len(session) != 16 or any(c not in "0123456789abcdef" for c in session):
        raise ValueError("invalid session")
    if not 1 <= sequence <= 0xFFFFFFFF or not 0 <= brightness <= 102:
        raise ValueError("sequence/brightness out of range")
    state = LightState(state)
    if clear and state == LightState.EMERGENCY:
        raise ValueError("cannot clear an active emergency")
    return envelope("S1,{},{},{},{},{}".format(session, sequence, int(state), brightness, int(clear)))


def valid_ack(frame, session, sequence):
    try:
        return unpack(frame) == ["A1", session, str(sequence)]
    except (ValueError, UnicodeError):
        return False


class SimulatedPort:
    """No device access: same wire path and ACK validation as real transport."""
    def __init__(self):
        self.frames = deque(maxlen=64)
        self.closed = False

    def write(self, frame):
        self.frames.append(frame)
        return len(frame)

    def readline(self, _size):
        fields = unpack(self.frames[-1])
        return envelope("A1,{},{}".format(fields[1], fields[2]))

    def close(self):
        self.closed = True


class IndicatorTransport:
    def __init__(self, factory, retry_seconds=2.0):
        self.factory = factory
        self.retry_seconds = retry_seconds
        self.port = None
        self.retry_at = 0.0
        self.session = ""
        self.sequence = 0
        self.last_error = ""

    def send(self, state, now, brightness=102, clear=False):
        if self.port is None and now < self.retry_at:
            return False
        try:
            if self.port is None:
                self.port = self.factory()
                self.session = secrets.token_hex(8)
                self.sequence = 0
            self.sequence += 1
            frame = state_frame(self.session, self.sequence, state, brightness, clear)
            if self.port.write(frame) != len(frame):
                raise IOError("short USB write")
            # One outstanding frame. Never queue/replay old colors on reconnect.
            if not valid_ack(self.port.readline(128), self.session, self.sequence):
                raise IOError("missing or invalid indicator ACK")
            self.last_error = ""
            return True
        except (OSError, ValueError) as exc:
            self.last_error = str(exc)
            self.close()
            self.retry_at = now + self.retry_seconds
            return False

    def close(self):
        if self.port is not None:
            try:
                self.port.close()
            except OSError:
                pass
            self.port = None
