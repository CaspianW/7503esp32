"""Firmware telemetry contract, independent of ROS and serial dependencies.

All calls must be protected by the bridge's condition lock. Source timestamps
are uint32 milliseconds; ROS stamps are assigned once, when a new sample arrives.
"""

from dataclasses import dataclass
import math


RECORD_TOKENS = {'S': 14, 'E': 13, 'O': 8, 'I': 9, 'L': 7, 'P': 9}
STATUS_FIELDS = (
    'initialized', 'armed', 'motion_allowed', 'emergency_stopped', 'fault',
    'encoder_ready', 'imu_ready', 'speed_ready', 'bench_mode',
    'motor_outputs_enabled', 'chassis_control_enabled', 'manual_mode',
)
UINT32_MASK = (1 << 32) - 1


def unsigned_ms(value):
    if not value.is_integer() or not 0 <= value <= UINT32_MASK:
        raise ValueError('Invalid firmware timestamp')
    return int(value)


@dataclass(frozen=True)
class Sample:
    source_ms: int
    received: float
    board_age_s: float
    stamp_ns: int
    values: tuple


class Telemetry:
    def __init__(self):
        self.generation = 0
        self.status_sequence = 0
        self.error_sequence = 0
        self.reset()

    def reset(self):
        self.generation += 1
        self.state = {}
        self.board_ms = None
        self.status_received = None
        self.samples = {}
        self.pending = set()

    def feed(self, line, now, ros_ns):
        """Accept a complete ASCII line; malformed records raise ValueError."""
        parts = line.split()
        if not parts:
            return None
        tag = parts[0]
        if tag == 'OK' and len(parts) == 1:
            return tag
        if tag == 'ERR':
            self.error_sequence += 1
            return tag
        if len(parts) != RECORD_TOKENS.get(tag):
            raise ValueError('Unknown telemetry tag or field count')
        values = [float(value) for value in parts[1:]]
        if not all(math.isfinite(value) for value in values):
            raise ValueError('Nonfinite telemetry')
        if tag == 'S':
            board_ms = unsigned_ms(values[0])
            state = dict(zip(STATUS_FIELDS, values[1:]))
            if any(value not in (0., 1.) for key, value in state.items() if key != 'fault'):
                raise ValueError('Invalid status flag')
            if not state['fault'].is_integer() or not 0 <= state['fault'] <= 8:
                raise ValueError('Invalid fault code')
            if self.board_ms is not None and \
                    (board_ms - self.board_ms) & UINT32_MASK > UINT32_MASK // 2:
                self.reset()  # Backward board clock, excluding millis() rollover.
            self.state = {key: int(value) for key, value in state.items()}
            self.board_ms, self.status_received = board_ms, now
            self.status_sequence += 1
            if not self.state['imu_ready']:
                self.samples.pop('imu', None)
                self.pending.discard('imu')
        elif tag in ('O', 'I'):
            kind = 'odom' if tag == 'O' else 'imu'
            source_ms = unsigned_ms(values[0])
            if not source_ms or self.board_ms is None:
                return tag  # Firmware's zero-initialized structs are not measurements.
            board_age = ((self.board_ms - source_ms) & UINT32_MASK) / 1000.
            if board_age > UINT32_MASK / 2000.:
                return tag
            old = self.samples.get(kind)
            if old is not None and source_ms == old.source_ms:
                return tag  # Repeated UART telemetry must not become a new sample.
            if old is not None and \
                    (source_ms - old.source_ms) & UINT32_MASK > UINT32_MASK // 2:
                return tag
            self.samples[kind] = Sample(source_ms, now, board_age,
                                        ros_ns - round(board_age * 1e9), tuple(values[1:]))
            self.pending.add(kind)
        elif tag == 'L':
            if any(value not in (0., 1.) for value in values[:4]) or \
                    values[4] not in (0., 1., 2.) or not values[5].is_integer() or \
                    not 0 <= values[5] <= 2500:
                raise ValueError('Invalid lift status')
            self.samples['lift'] = Sample(0, now, 0., ros_ns, tuple(values))
            self.pending.add('lift')
        return tag

    def status_fresh(self, now, limit):
        return self.status_received is not None and 0 <= now - self.status_received <= limit

    def fresh(self, kind, now, limit):
        sample = self.samples.get(kind)
        if sample is None or sample.stamp_ns <= 0 or not self.status_fresh(now, limit) or \
                not self.state.get('initialized'):
            return False
        if now - sample.received + sample.board_age_s > limit:
            return False
        if kind == 'imu' and not self.state.get('imu_ready'):
            return False
        if kind == 'odom' and not (self.state.get('encoder_ready') and
                                   self.state.get('speed_ready')):
            return False
        if kind != 'lift':
            age = ((self.board_ms - sample.source_ms) & UINT32_MASK) / 1000.
            if age > limit:
                return False
        return True

    def motion_ready(self, now, limit):
        return (self.fresh('odom', now, limit) and self.fresh('imu', now, limit)
                and self.state.get('fault') == 0
                and not self.state.get('emergency_stopped')
                and not self.state.get('bench_mode')
                and not self.state.get('manual_mode')
                and self.state.get('motor_outputs_enabled') == 1
                and self.state.get('chassis_control_enabled') == 1)

    def take_pending(self, now, limit):
        result = {kind: self.samples[kind] for kind in self.pending
                  if self.fresh(kind, now, limit)}
        self.pending.clear()
        return result
