"""Generate only v2 geometry/hover overrides; retain the shared v1 EKF/gains."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from vehicle_profiles import SIMULATOR, load_profile, px4_overrides


def generate():
    profile=load_profile('argo_v2')
    lines=['#!/bin/sh','# @name argo_v2 Isaac Sim SITL','# @type Quadrotor x',
           '. ${R}etc/init.d-posix/airframes/10020_isaac_argo',
           '# Generated from simulator/config/vehicles/argo_v2.json.']
    lines += [f'param set {key} {value:.12g}' for key,value in px4_overrides(profile).items()]
    target=SIMULATOR/'px4/10021_isaac_argo_v2'
    target.write_text('\n'.join(lines)+'\n')
    return target


if __name__=='__main__':
    print(generate())
