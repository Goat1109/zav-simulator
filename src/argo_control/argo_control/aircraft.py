"""Resolve selected simulator aircraft properties for ROS console entry points."""
from functools import lru_cache
import importlib.util
import os
from pathlib import Path


@lru_cache(maxsize=1)
def imu_mount():
    root=Path(os.environ.get('ARGO_WS_DIR',Path(__file__).resolve().parents[3]))
    spec=importlib.util.spec_from_file_location('argo_control_profiles',root/'simulator/vehicle_profiles.py')
    profiles=importlib.util.module_from_spec(spec);spec.loader.exec_module(profiles)
    return profiles.imu_mount()
