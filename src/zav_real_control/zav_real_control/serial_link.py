"""Bounded serial I/O. A broken flight link requires a ground-side restart."""
from pymavlink import mavutil
from pathlib import Path


def configure_ftdi_latency(device, milliseconds, sysfs_root='/sys/class/tty'):
    """Configure only this port's FTDI receive latency; no serial/FC messages.

    Resolve by-id links on every startup, including after USB re-enumeration.
    Other UART drivers have no FTDI knob and are left unchanged. A known FTDI
    configuration failure aborts startup instead of silently retaining 16 ms.
    """
    if type(milliseconds) is not int or not 1<=milliseconds<=255:
        raise ValueError('FTDI latency must be an integer in 1..255 ms')
    tty=Path(device).resolve().name
    port=Path(sysfs_root)/tty/'device'
    if (port/'driver').resolve().name!='ftdi_sio':
        return None
    timer=port/'latency_timer'
    try:
        before=int(timer.read_text().strip())
        if before!=milliseconds:
            timer.write_text(f'{milliseconds}\n')
        actual=int(timer.read_text().strip())
        if actual!=milliseconds:
            raise OSError(f'read-back was {actual} ms')
    except (OSError,ValueError) as error:
        raise OSError(f'Cannot set FTDI latency to {milliseconds} ms at {timer}: {error}. '
                      'Start the bridge with sysfs write access (argo container root).') from error
    return dict(device=tty,previous_ms=before,actual_ms=actual)


class CheckedSerial(mavutil.mavserial):
    def __init__(self, *args, **kwargs):
        kwargs['autoreconnect'] = False
        super().__init__(*args, **kwargs)
        self.failure = None
        # timeout=0 in mavserial only bounds reads, not writes.
        self.port.write_timeout = .020

    def _check(self):
        if self.failure:
            raise OSError(self.failure)

    def _fail(self, error):
        self.failure = f'MAVLink serial failure; ground restart required: {error}'
        raise OSError(self.failure)

    def write(self, buf):
        self._check()
        packet = bytes(buf)
        try:
            written = self.port.write(packet)
        except Exception as error:
            # mavserial.write otherwise swallows the error and returns -1.
            self._fail(error)
        if written != len(packet):
            self._fail(f'short write {written}/{len(packet)} bytes')
        return written

    def recv(self, n=None):
        self._check()
        try:
            return super().recv(n)
        except Exception as error:
            self._fail(error)
