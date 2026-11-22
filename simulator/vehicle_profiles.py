"""Aircraft properties shared by Isaac, ROS launch, PX4 and flight records.

Lengths are metres in body FLU; mount Euler angles are ZYX degrees.
No ROS/Isaac imports: this module works in both Python environments.
"""
import argparse
import copy
import importlib.util
import json
import math
import os
from pathlib import Path

SIMULATOR = Path(__file__).resolve().parent
VEHICLES = ('argo_v1', 'argo_v2')


def load_profile(name=None):
    name = name or os.environ.get('ARGO_VEHICLE', 'argo_v1')
    if name not in VEHICLES:
        raise ValueError(f'Unknown aircraft {name!r}; choose argo_v1 or argo_v2')
    data = json.loads((SIMULATOR / 'config/vehicles' / f'{name}.json').read_text())
    if data['name'] != name or data['mass'] <= 0:
        raise ValueError(f'Invalid aircraft profile: {name}')
    return data


def import_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def active_profile(requested=None):
    """Consumers inherit the live simulator, rejecting cross-aircraft feedback."""
    sessions = import_file('argo_profile_session', SIMULATOR / 'tools/runtime/sim_session.py')
    session = sessions.read_session()
    requested = requested or os.environ.get('ARGO_VEHICLE')
    if session:
        if requested and requested != session['vehicle']:
            raise ValueError(f"Simulator is {session['vehicle']}, requested {requested}")
        requested = session['vehicle']
        if session.get('calibration_dir'):
            os.environ['PEGASUS_FASTLIVO_CONFIG_DIR'] = session['calibration_dir']
    return load_profile(requested)


def calibration_dir(profile):
    return Path(os.environ.get('PEGASUS_FASTLIVO_CONFIG_DIR') or
                SIMULATOR / profile['calibration_dir']).expanduser().resolve()


def sensor_calibration(profile):
    module = import_file('argo_mount_calibration',
                         SIMULATOR / 'pegasus/simulator/logic/sensors/fastlivo_calibration.py')
    return module.load_calibration(calibration_dir(profile), profile['lidar_position'],
                                   profile['lidar_orientation'])


def imu_mount(profile=None):
    from scipy.spatial.transform import Rotation
    mounts = sensor_calibration(profile or active_profile())
    q = Rotation.from_euler('ZYX', mounts['imu_orientation'], degrees=True).as_quat()
    return q[[3, 0, 1, 2]].tolist(), mounts['imu_position']


def navigation_config(base, profile):
    result = copy.deepcopy(base)
    result['traj_opt']['flatness']['mass'] = profile['mass']
    # An experimental larger margin must never be reduced by aircraft selection.
    result['super_planner']['robot_r'] = max(result['super_planner']['robot_r'], profile['robot_radius'])
    p = profile['propulsion']
    result['argo_ommpc_px4']['ros__parameters'].update(
        mass=profile['mass'], rotor_constant=p['rotor_constant'], rotor_scale=p['rotor_scale'],
        rotor_idle=p['rotor_idle'], hover_throttle=profile['hover_throttle'])
    return result


def total_com(profile):
    if profile['name'] == 'argo_v1':
        return profile['body_com']  # Preserve v1 allocation convention.
    return [(profile['body_mass']*profile['body_com'][j] + profile['rotor_mass']*
             sum(p[j] for p in profile['rotor_coms'])) / profile['mass'] for j in range(3)]


def px4_overrides(profile):
    if profile['px4_geometry'] == 'legacy':
        positions = [[.15,.15,0.],[-.15,-.15,0.],[.15,-.15,0.],[-.15,.15,0.]]
        moments = [.05,.05,-.05,-.05]
    else:
        com = total_com(profile)
        positions = [[p[0]-com[0], -(p[1]-com[1]), -(p[2]-com[2])]
                     for p in profile['rotor_axes']]
        # Preserve v1 motor/propeller parameters, including PX4 allocation KM.
        moments = [.05,.05,-.05,-.05]
    values = {'MPC_THR_HOVER':profile['hover_throttle'], 'CA_ROTOR_COUNT':4}
    for i,(position,moment) in enumerate(zip(positions,moments)):
        for axis,value in zip('XYZ',position):
            values[f'CA_ROTOR{i}_P{axis}'] = value
        values[f'CA_ROTOR{i}_KM'] = moment
    return values


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=('check','show','calibration-dir'))
    p.add_argument('--vehicle',choices=VEHICLES)
    args = p.parse_args()
    profile = load_profile(args.vehicle)
    if args.action == 'calibration-dir':
        print(calibration_dir(profile))
    elif args.action == 'show':
        print(json.dumps(profile,indent=2))
    else:
        if not (SIMULATOR / profile['usd']).is_file():
            p.error(f"Missing aircraft USD: {profile['usd']}")
        if not (calibration_dir(profile)/'zenith.yaml').is_file():
            p.error(f'Missing calibration: {calibration_dir(profile)}')
        print(profile['name'])


if __name__ == '__main__':
    main()
