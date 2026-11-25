"""Named int32 PX4 parameter protocol; no sockets here (unit-testable)."""
import struct

PARAMETER = 'EKF2_BARO_CTRL'
INT32 = 6


def encode(value):
    return struct.unpack('<f', struct.pack('<i', value))[0]


def decode(value):
    return struct.unpack('<i', struct.pack('<f', value))[0]


class ParameterLink:
    def __init__(self, connection, expected_version):
        self.connection = connection
        self.expected_version = tuple(expected_version)
        self.target = None
        self.value = self.value_rx = self.heartbeat_rx = None
        self.version = None
        self.last_request = self.last_version_request = -float('inf')
        self.set_count = 0

    @property
    def supported(self):
        return self.version == self.expected_version

    def poll(self, now):
        for _ in range(100):
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                return
            if self.target is None or (msg.get_srcSystem(),msg.get_srcComponent()) != self.target:
                continue
            kind = msg.get_type()
            if kind == 'HEARTBEAT':
                self.heartbeat_rx = now
            elif kind == 'AUTOPILOT_VERSION':
                v = int(msg.flight_sw_version)
                self.version = ((v>>24)&255, (v>>16)&255, (v>>8)&255)
            elif kind == 'PARAM_VALUE' and msg.param_type == INT32:
                name = msg.param_id.decode() if isinstance(msg.param_id,bytes) else msg.param_id
                if name.rstrip('\0') == PARAMETER:
                    value = decode(msg.param_value)
                    if value in (0,1):
                        self.value, self.value_rx = value, now

    def tick(self, desired, armed, now, allow_writes):
        self.poll(now)
        if self.target is None or self.heartbeat_rx is None or now-self.heartbeat_rx > 3:
            return
        system, component = self.target
        if self.version is None and now-self.last_version_request >= 1:
            # MAV_CMD_REQUEST_MESSAGE / AUTOPILOT_VERSION, not a flight command.
            self.connection.mav.command_long_send(system,component,512,0,148,0,0,0,0,0,0)
            self.last_version_request = now
        if now-self.last_request < .5:
            return
        if (allow_writes and self.supported and self.value != desired
                and (desired == 1 or not armed)):
            self.connection.mav.param_set_send(system,component,PARAMETER.encode(),encode(desired),INT32)
            self.set_count += 1
        self.connection.mav.param_request_read_send(system,component,PARAMETER.encode(),-1)
        self.last_request = now

    def snapshot(self, desired, now):
        fresh = self.value_rx is not None and now-self.value_rx < 2
        return dict(parameter=PARAMETER, desired=desired, value=self.value,
                    confirmed=fresh and self.value==desired, target=list(self.target) if self.target else None,
                    firmware=list(self.version) if self.version else None, firmware_supported=self.supported,
                    value_age_s=now-self.value_rx if self.value_rx is not None else None, set_count=self.set_count)
