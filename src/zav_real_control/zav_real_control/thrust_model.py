"""Fit body-Z specific force against the thrust actually written to the FC."""
import math
from collections import deque


class ThrustModel:
    gravity = 9.81
    forgetting = .998

    def __init__(self):
        self.clock = deque(maxlen=32)
        self.reset()

    def reset(self):
        self.coefficient = None
        self.covariance = 1e6
        self.commands = deque(maxlen=100)
        self.updates = 0
        self.last_sample = 0
        self.updated_at = None

    def sync(self, boot_ns, now):
        # PX4 originates these TIMESYNC requests using its monotonic clock.
        # Use the least delayed recent observation of FC time on our clock.
        if boot_ns <= 0:
            return
        offset = now - boot_ns * 1e-9
        if self.clock and abs(offset - self.clock[-1][1]) > .1:
            self.clock.clear()
            self.commands.clear()
        self.clock.append((now, offset))

    def sample(self, msg, now):
        if (self.coefficient is None or not self.commands or not msg.fields_updated & 4
                or getattr(msg, 'id', 0) != 0 or msg.time_usec <= self.last_sample):
            return
        self.last_sample = msg.time_usec
        offsets = [offset for at, offset in self.clock if 0 <= now-at <= 1.]
        if not offsets:
            return
        measured = msg.time_usec * 1e-6 + min(offsets)
        az = -msg.zacc  # HIGHRES_IMU is FRD m/s²; OMMPC uses FLU specific force.
        if not (-.003 <= now-measured <= .05 and math.isfinite(az) and az > 0):
            return
        while self.commands and measured-self.commands[0][0] > .045:
            self.commands.popleft()
        if not self.commands or measured-self.commands[0][0] < .035:
            return
        _, thrust = self.commands.popleft()
        gain = self.covariance*thrust / (self.forgetting + self.covariance*thrust**2)
        candidate = self.coefficient + gain*(az-thrust*self.coefficient)
        self.coefficient = min(self.gravity/.1, max(self.gravity/.8, candidate))
        self.covariance = (1-gain*thrust)*self.covariance/self.forgetting
        self.updates += 1
        self.updated_at = now
