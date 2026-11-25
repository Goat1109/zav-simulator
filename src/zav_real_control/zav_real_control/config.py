import math
from pathlib import Path
import yaml
from .geometry import unit


def load_config(path):
    with Path(path).expanduser().open() as stream:
        c = yaml.safe_load(stream)
    if not isinstance(c, dict):
        raise ValueError('hardware.yaml must contain a mapping')
    transport=c.get('transport','dds')
    if transport not in ('mavlink','dds'):raise ValueError('transport must be mavlink or dds')
    firmware_key='firmware_verified' if transport=='mavlink' else 'firmware_abi_verified'
    for name in ('publish_enabled',firmware_key,'calibration_verified','respect_rc_takeover'):
        if type(c[name]) is not bool:
            raise ValueError(f'{name} must be true/false')
    unit(c['body_from_imu_q_wxyz'])
    if len(c['imu_position_body_flu_m']) != 3 or not all(math.isfinite(x) for x in c['imu_position_body_flu_m']):
        raise ValueError('Invalid IMU lever arm')
    for name in ('position_variance_m2','orientation_variance_rad2','velocity_variance_m2_s2'):
        if len(c[name]) != 3 or not all(math.isfinite(x) and x > 0 for x in c[name]):
            raise ValueError(f'Invalid {name}')
    for name in ('rate_hz','max_odom_age_s','max_feedback_age_s','max_timesync_age_s',
                 'baro_activation_timeout_s'):
        if not math.isfinite(c[name]) or c[name] <= 0:
            raise ValueError(f'{name} must be positive')
    if not 10 <= c['rate_hz'] <= 100:
        raise ValueError('rate_hz must be 10..100')
    if len(c['expected_firmware']) != 3 or not all(type(v) is int and 0 <= v < 256 for v in c['expected_firmware']):
        raise ValueError('expected_firmware must be [major,minor,patch]')
    for key in ('source_system','source_component'):
        if type(c[key]) is not int or not 1 <= c[key] <= 255:
            raise ValueError(f'{key} must be 1..255')
    if transport=='mavlink':
        link=c['mavlink']
        link.setdefault('timesync_initial_rate_hz',100)
        link.setdefault('timesync_steady_rate_hz',10)
        link.setdefault('ftdi_latency_timer_ms',1)
        for key,low,high in [('timesync_initial_rate_hz',10,100),
                             ('timesync_steady_rate_hz',1,100),
                             ('ftdi_latency_timer_ms',1,255)]:
            value=link[key]
            if type(value) is not int or not low<=value<=high:
                raise ValueError(f'Invalid mavlink.{key}')
        if link['timesync_steady_rate_hz']>link['timesync_initial_rate_hz']:
            raise ValueError('Steady timesync rate must not exceed initial rate')
        heartbeat=c['mavlink']['heartbeat_timeout_s']
        if not math.isfinite(heartbeat) or not 1<=heartbeat<=10:raise ValueError('Invalid heartbeat timeout')
        for key,low,high in [('baud',9600,3000000),('target_system',1,255),('target_component',1,255),
                             ('rc_mode_channel',0,18),('rc_switch_delta_us',50,1000)]:
            value=c['mavlink'][key]
            if type(value) is not int or not low<=value<=high:raise ValueError(f'Invalid mavlink.{key}')
        if not isinstance(c['mavlink']['connection'],str) or not c['mavlink']['connection'].strip():
            raise ValueError('Set mavlink.connection')
        if type(c['ros_domain_id']) is not int or not 0<=c['ros_domain_id']<=232:raise ValueError('Invalid ROS domain')
        return c
    if c['agent'].get('transport') not in ('serial','udp4'):
        raise ValueError('agent.transport must be serial or udp4')
    if c['agent']['transport']=='serial' and c['agent']['device']==c['mavlink']['connection']:
        raise ValueError('DDS and MAVLink must not share one UART')
    return c
