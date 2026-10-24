"""Per-wheel manual reversal transitions, independent of ROS (Python 3.8)."""
from dataclasses import dataclass
import math
import threading


def sign(value):
    return 1 if value > 0 else -1 if value < 0 else 0


def approach(value, target, step):
    return min(target, value + step) if target > value else max(target, value - step)


@dataclass
class Wheel:
    output: int = 0
    last_sign: int = 0
    zero_since: float = None
    zero_ros: float = None
    sample_stamp: float = None
    stopped_samples: int = 0
    accelerating: bool = False


class ManualWheelTransition:
    def __init__(self, dwell_sec, feedback_timeout=0.5):
        self.dwell = max(0.0, float(dwell_sec))
        self.feedback_timeout = max(0.1, float(feedback_timeout))
        self.lock = threading.RLock()
        self.reset()

    def reset(self):
        with self.lock:
            self.wheels = [Wheel(), Wheel()]
            self.last_update = None
            self.ramp_fraction = 0.0

    def observe(self, stamp, now_ros, left, right, target_left, target_right, valid):
        with self.lock:
            fresh = (valid and math.isfinite(stamp) and math.isfinite(now_ros)
                     and 0.0 <= now_ros - stamp <= self.feedback_timeout)
            for state, measured, target in zip(self.wheels, (left, right), (target_left, target_right)):
                if (not fresh or state.zero_ros is None or stamp <= state.zero_ros
                        or measured != 0 or target != 0):
                    state.stopped_samples = 0
                    continue
                if state.sample_stamp is None or stamp > state.sample_stamp:
                    state.sample_stamp = stamp
                    state.stopped_samples += 1

    def _commit(self, outputs, now, now_ros):
        for state, value in zip(self.wheels, outputs):
            value = int(value)
            if value:
                state.last_sign = sign(value)
                state.zero_since = state.zero_ros = state.sample_stamp = None
                state.stopped_samples = 0
            elif state.output or state.zero_since is None:
                state.zero_since, state.zero_ros = now, now_ros
                state.sample_stamp, state.stopped_samples = None, 0
            state.output = value
        self.last_update = now
        return tuple(int(v) for v in outputs)

    def apply(self, left, right, now, now_ros, max_rpm):
        with self.lock:
            goals = (int(left), int(right))
            limit = max(1, int(max_rpm))
            if goals == (0, 0):
                for state in self.wheels:
                    state.accelerating = False
                return self._commit((0, 0), now, now_ros)
            if self.dwell <= 0:
                return self._commit(goals, now, now_ros)

            reversing = [bool(value and state.last_sign and sign(value) != state.last_sign)
                         for state, value in zip(self.wheels, goals)]
            both_reverse = all(reversing)
            both_ready = all(state.zero_since is not None and now - state.zero_since >= self.dwell
                             for state in self.wheels)
            dt = 0.1 if self.last_update is None else min(0.1, max(0.0, now - self.last_update))
            budget = self.ramp_fraction + limit * dt / max(0.1, self.dwell)
            step = int(budget + 1e-9)
            self.ramp_fraction = max(0.0, budget - step)
            outputs, transitioning = [], []
            for index, (state, target) in enumerate(zip(self.wheels, goals)):
                if not target:
                    state.accelerating = False
                    outputs.append(0)
                    continue
                if reversing[index]:
                    state.accelerating = False
                    transitioning.append(index)
                    waited = state.zero_since is not None and now - state.zero_since >= self.dwell
                    confirmed = (state.stopped_samples >= 2 and state.sample_stamp is not None
                                 and 0.0 <= now_ros - state.sample_stamp <= self.feedback_timeout)
                    if both_reverse and not both_ready:
                        output = 0  # Keep the original full stop for a two-wheel reversal.
                    elif state.output:
                        output = approach(state.output, 0, step)
                    elif waited or (confirmed and not both_reverse):
                        output = approach(0, target, step)
                        state.accelerating = True
                    else:
                        output = 0
                elif state.accelerating:
                    transitioning.append(index)
                    output = approach(state.output, target, step)
                    if output == target:
                        state.accelerating = False
                else:
                    output = target
                outputs.append(int(output))

            if len(transitioning) == 1 and not both_reverse:
                # Preserve the requested forward component while the inner
                # wheel brakes/holds/crosses zero; do not stop the other wheel.
                index = transitioning[0]
                other = 1 - index
                requested_sum = sum(goals)
                other_limit = min(limit, abs(goals[other]))
                low, high = (-other_limit, 0) if goals[other] < 0 else (0, other_limit) if goals[other] > 0 else (0, 0)
                other_output = max(low, min(high, requested_sum - outputs[index]))
                current = outputs[index]
                combined = current + other_output
                within_longitudinal_bound = (combined == 0 or
                    (requested_sum != 0 and sign(combined) == sign(requested_sum)
                     and abs(combined) <= abs(requested_sum)))
                if within_longitudinal_bound:
                    # A slower translation is valid while the wheel ramps;
                    # do not erase sub-target progress trying to match exactly.
                    outputs[other] = other_output
                else:
                    adjusted = requested_sum - other_output
                    if min(0, current) <= adjusted <= max(0, current):
                        outputs[index], outputs[other] = adjusted, other_output
                    else:
                        # Full steering requests zero translation. Safety takes
                        # priority over residual straight motion during reversal.
                        outputs[index] = 0
                        outputs[other] = max(low, min(high, requested_sum))

            # Rounding and transitions must never increase the requested
            # longitudinal component or produce motion in its opposite sign.
            desired_sum, actual_sum = sum(goals), sum(outputs)
            if actual_sum and (not desired_sum or sign(actual_sum) != sign(desired_sum)):
                outputs = [0, 0]
            elif abs(actual_sum) > abs(desired_sum):
                ratio = abs(desired_sum / actual_sum)
                outputs = [int(v * ratio) for v in outputs]
            outputs = [max(-limit, min(limit, int(value))) for value in outputs]
            return self._commit(outputs, now, now_ros)
