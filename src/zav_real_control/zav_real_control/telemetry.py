"""Bounded, read-only monitoring data; never participates in flight decisions."""
from collections import deque


class FlightTelemetry:
    def __init__(self):
        self.rc7 = None
        self.rc7_rx = None
        self.battery = None
        self.battery_rx = None
        self.fc_state = None
        self.texts = deque(maxlen=5)
        self.odom = deque(maxlen=2000)
        self.last_odom = None
        self.cached = None
        self.cached_at = -float("inf")

    def receive(self, msg, now):
        kind = msg.get_type()
        if kind == 'RC_CHANNELS':
            value=getattr(msg,'chan7_raw',65535)
            self.rc7=value if 800<=value<=2200 else None
            self.rc7_rx=now if self.rc7 is not None else None
            self.cached=None
        if kind == 'BATTERY_STATUS' and msg.id == 0:
            volts = [v for v in msg.voltages if 0 < v < 65535]
            volts += [v for v in getattr(msg, 'voltages_ext', []) if 0 < v < 65535]
            self.battery = dict(voltage_v=sum(volts)/1000 if volts else None,
                                remaining_pct=msg.battery_remaining if 0 <= msg.battery_remaining <= 100 else None,
                                charge_state=getattr(msg, 'charge_state', 0))
            self.battery_rx = now
        elif kind == 'HEARTBEAT':
            self.fc_state = getattr(msg, 'system_status', None)
        elif kind == 'STATUSTEXT' and msg.severity <= 4:
            raw = msg.text
            text = raw.decode(errors='replace') if isinstance(raw, bytes) else raw
            text = ''.join(c for c in text.rstrip('\0') if c.isprintable())
            self.texts.append(dict(text=text, severity=msg.severity, rx=now))

    def odometry(self, now, stamp_us, epoch_us):
        age = (epoch_us-stamp_us)/1000
        self.odom.append((now, age))
        self.last_odom = (now, age)

    def snapshot(self, now):
        if self.cached is not None and now-self.cached_at < .2:
            return self.cached
        self.cached_at = now
        while self.odom and now-self.odom[0][0] > 2:
            self.odom.popleft()
        values = sorted(age for _, age in self.odom)
        hz = ((len(self.odom)-1)/(self.odom[-1][0]-self.odom[0][0])
              if len(self.odom) > 1 and self.odom[-1][0] > self.odom[0][0] else 0.)
        battery_age = now-self.battery_rx if self.battery_rx is not None else None
        rc_age=now-self.rc7_rx if self.rc7_rx is not None else None
        self.cached = dict(rc7=dict(pwm=self.rc7,age_s=rc_age,fresh=rc_age is not None and 0<=rc_age<.5),battery=self.battery, battery_age_s=battery_age,
                    battery_fresh=battery_age is not None and 0 <= battery_age < 3,
                    fc_system_status=self.fc_state,
                    odometry=dict(hz=hz, age_median_ms=values[len(values)//2] if values else None,
                                  age_max_ms=max(values) if values else None,
                                  last_measurement_age_ms=self.last_odom[1]+(now-self.last_odom[0])*1000 if self.last_odom else None),
                    messages=[dict(text=t['text'], age_s=now-t['rx']) for t in self.texts])

        return self.cached
