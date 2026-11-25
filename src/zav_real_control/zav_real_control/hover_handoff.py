"""Learn one entry's hover scale from PX4's stable Position thrust output.

No I/O. ATTITUDE_TARGET is read on the existing UART, never from our own
SET_ATTITUDE_TARGET output. A frozen scale cannot learn its own MPC correction.
"""
from collections import deque
import math
import statistics


class HoverHandoff:
    WINDOW = 1.5
    MIN_SPAN = 1.0
    MIN_SAMPLES = 20
    MAX_GAP = .25
    MAX_SPREAD = .04

    def __init__(self):
        self.reset()

    def reset(self):
        self.samples = deque(maxlen=80)
        self.value = None
        self.ready = self.frozen = False
        self.frozen_at = None
        self.reason = 'waiting_for_position_hover'
        self.last_boot_ms = None

    def sample(self, now, boot_ms, thrust, tilt_cos):
        if self.frozen:
            return
        # PX4 repeats its old attitude target during body-rate Offboard. Require
        # a new producer timestamp, not just a newly received MAVLink packet.
        if self.last_boot_ms is not None and boot_ms <= self.last_boot_ms:
            return
        self.last_boot_ms = boot_ms
        if not (.1 <= thrust <= .8 and math.cos(math.radians(10)) <= tilt_cos <= 1.):
            self.clear('position_thrust_or_tilt_out_of_range')
            return
        if self.samples and now-self.samples[-1][0] > self.MAX_GAP:
            self.clear('position_thrust_gap')
        self.samples.append((now, thrust*tilt_cos))
        while self.samples and now-self.samples[0][0] > self.WINDOW:
            self.samples.popleft()
        values = [v for _,v in self.samples]
        self.ready = (len(values) >= self.MIN_SAMPLES
                      and now-self.samples[0][0] >= self.MIN_SPAN-1e-6
                      and max(values)-min(values) <= self.MAX_SPREAD)
        self.value = statistics.median(values) if self.ready else None
        self.reason = 'position_attitude_target' if self.ready else 'collecting_stable_position_thrust'

    def clear(self, reason):
        if self.frozen:
            return
        self.samples.clear()
        self.value = None
        self.ready = False
        self.reason = reason

    def fresh(self, now):
        return self.ready and (self.frozen or bool(self.samples) and 0 <= now-self.samples[-1][0] < self.MAX_GAP)

    def freeze(self, now):
        if not self.fresh(now):
            return False
        if not self.frozen:self.frozen_at = now
        self.frozen = True
        return True

    def snapshot(self, now):
        return dict(ready=self.fresh(now), frozen=self.frozen, value=self.value,
                    source=self.reason, samples=len(self.samples),
                    age_s=now-self.samples[-1][0] if self.samples else None)
