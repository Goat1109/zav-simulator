from types import SimpleNamespace

import pytest
pytest.importorskip('pymavlink')
from pymavlink import mavutil
from zav_real_control.serial_link import CheckedSerial, configure_ftdi_latency


def connection(monkeypatch, write):
    def init(self, *args, **kwargs):
        assert kwargs['autoreconnect'] is False
        self.port = SimpleNamespace(write=write)
    monkeypatch.setattr(mavutil.mavserial, '__init__', init)
    return CheckedSerial('/unused')


def test_full_write_and_bounded_timeout(monkeypatch):
    c = connection(monkeypatch, lambda b: len(b))
    assert c.port.write_timeout == .020
    assert c.write(b'packet') == 6


@pytest.mark.parametrize('result', [-1, 0, 2, None])
def test_failed_or_partial_write_is_sticky(monkeypatch, result):
    c = connection(monkeypatch, lambda b: result)
    with pytest.raises(OSError, match='short write'):
        c.write(b'packet')
    c.port.write = lambda b: len(b)
    with pytest.raises(OSError, match='ground restart required'):
        c.write(b'packet')
    with pytest.raises(OSError, match='ground restart required'):
        c.recv()


def test_timeout_propagates_instead_of_minus_one(monkeypatch):
    def timeout(b):
        raise TimeoutError('write timeout')
    c = connection(monkeypatch, timeout)
    with pytest.raises(OSError, match='write timeout'):
        c.write(b'packet')


def test_read_failure_is_sticky(monkeypatch):
    c = connection(monkeypatch, lambda b: len(b))
    def broken_read(self, n):
        raise OSError('USB disconnected')
    monkeypatch.setattr(mavutil.mavserial, 'recv', broken_read)
    with pytest.raises(OSError, match='USB disconnected'):
        c.recv()
    with pytest.raises(OSError, match='ground restart required'):
        c.write(b'packet')


def ftdi_port(tmp_path):
    port=tmp_path/'sys'/'ttyUSB9'/'device'
    port.mkdir(parents=True)
    (port/'driver').symlink_to(tmp_path/'drivers'/'ftdi_sio')
    timer=port/'latency_timer'
    timer.write_text('16\n')
    device=tmp_path/'by-id'/'usb-FTDI'
    device.parent.mkdir()
    device.symlink_to(tmp_path/'dev'/'ttyUSB9')
    return device,timer,tmp_path/'sys'


def test_ftdi_follows_by_id_and_verifies_one_ms(tmp_path):
    device,timer,root=ftdi_port(tmp_path)
    result=configure_ftdi_latency(device,1,root)
    assert result==dict(device='ttyUSB9',previous_ms=16,actual_ms=1)
    assert timer.read_text()=='1\n'
    assert configure_ftdi_latency(device,1,root)['previous_ms']==1


def test_other_serial_driver_is_not_modified(tmp_path):
    device,timer,root=ftdi_port(tmp_path)
    driver=timer.parent/'driver';driver.unlink()
    driver.symlink_to(tmp_path/'drivers'/'cp210x')
    assert configure_ftdi_latency(device,1,root) is None
    assert timer.read_text()=='16\n'


def test_missing_ftdi_knob_aborts_startup(tmp_path):
    device,timer,root=ftdi_port(tmp_path);timer.unlink()
    with pytest.raises(OSError,match='Cannot set FTDI latency'):
        configure_ftdi_latency(device,1,root)


def test_ftdi_readback_failure_is_not_silent(tmp_path,monkeypatch):
    device,timer,root=ftdi_port(tmp_path)
    from pathlib import Path
    monkeypatch.setattr(Path,'write_text',lambda *a,**kw:None)
    with pytest.raises(OSError,match='read-back was 16'):
        configure_ftdi_latency(device,1,root)


@pytest.mark.parametrize('value',[0,256,True,1.5])
def test_invalid_ftdi_latency_never_touches_sysfs(tmp_path,value):
    device,timer,root=ftdi_port(tmp_path)
    with pytest.raises(ValueError):configure_ftdi_latency(device,value,root)
    assert timer.read_text()=='16\n'
