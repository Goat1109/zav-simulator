"""No ROS and no I/O: conditional baro handoff and pilot takeover policy."""

class Handoff:
    MANUAL_INTENTIONS = (0, 1, 15)  # Manual, Altitude, Stabilized; never return to Position on bad EV.

    def __init__(self, timeout=3., respect_rc=True):
        self.timeout, self.respect_rc = timeout, respect_rc
        self.started = None
        self.initial_intention = None
        self.confirmed = False
        self.confirm_seconds = None
        self.timed_out = False
        self.pilot_takeover = False
        self.last_land = -float('inf')

    def update(self, now, latched, armed, status_fresh, nav_state, intention,
               parameter_on, baro_good, vertical_good):
        if latched and self.started is None:
            self.started, self.initial_intention = now, intention
        if self.started is not None:
            if (self.respect_rc and status_fresh and armed
                    and intention in self.MANUAL_INTENTIONS and intention != self.initial_intention):
                self.pilot_takeover = True
            if parameter_on and baro_good and vertical_good and not self.confirmed:
                self.confirmed, self.confirm_seconds = True, now-self.started
            if not self.confirmed and now-self.started >= self.timeout:
                self.timed_out = True
        send_land = (latched and armed and status_fresh and not self.pilot_takeover
                     and (self.confirmed or self.timed_out) and nav_state not in (12,18)
                     and now-self.last_land >= 1.)
        if send_land:
            self.last_land = now
        phase = ('PILOT_TAKEOVER' if self.pilot_takeover else
                 'BARO_ACTIVE' if self.confirmed else 'BARO_TIMEOUT' if self.timed_out else
                 'ENABLING_BARO' if latched else 'NORMAL_EV_ONLY')
        return dict(phase=phase, send_land=send_land, enable_baro=latched,
                    baro_confirmed=self.confirmed, baro_activation_seconds=self.confirm_seconds,
                    baro_timeout=self.timed_out, pilot_takeover=self.pilot_takeover)
