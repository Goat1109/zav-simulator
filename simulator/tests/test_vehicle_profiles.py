"""Aircraft-switch and reference-frame regressions, independent of Isaac/ROS."""
import copy
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import vehicle_profiles as profiles


def test_v2_mass_units_motor_parity_and_hover_force():
    v=profiles.load_profile('argo_v2');p=v['propulsion']
    assert v['mass']==pytest.approx((2538.596+4*9.138)*.001)
    assert p==profiles.load_profile('argo_v1')['propulsion']
    omega=v['hover_throttle']*p['rotor_scale']+p['rotor_idle']
    assert 4*p['rotor_constant']*omega**2==pytest.approx(v['mass']*9.81)
    assert np.linalg.eigvalsh(v['body_inertia']).min()>0


def test_camera_extrinsics_agree_with_cad_pose_and_horizontal_lidar():
    with patch.dict(os.environ,{},clear=True):
        v=profiles.load_profile('argo_v2');mounts=profiles.sensor_calibration(v)
        assert mounts['camera_position']==pytest.approx(v['camera_position'])
        assert Rotation.from_euler('ZYX',mounts['camera_orientation'],degrees=True).as_matrix()==pytest.approx(np.eye(3))
        assert mounts['imu_orientation']==pytest.approx([0.,0.,0.])
        assert profiles.sensor_calibration(profiles.load_profile('argo_v1'))['imu_orientation']==pytest.approx([0.,22.,0.])


def test_aircraft_overlay_preserves_experiment_and_larger_clearance():
    source=yaml.safe_load((profiles.SIMULATOR.parent/'src/argo_super/config/common.yaml').read_text())
    source['super_planner']['robot_r']=.7;saved=copy.deepcopy(source)
    result=profiles.navigation_config(source,profiles.load_profile('argo_v2'))
    assert source==saved
    assert result['traj_opt']['flatness']['mass']==pytest.approx(2.575148)
    assert result['super_planner']['robot_r']==.7
    assert result['traj_opt']['boundary']==source['traj_opt']['boundary']
    assert result['traj_opt']['exp_traj']==source['traj_opt']['exp_traj']


def test_px4_geometry_uses_cad_axes_relative_to_combined_com_in_frd():
    v=profiles.load_profile('argo_v2');values=profiles.px4_overrides(v);com=profiles.total_com(v)
    for i,axis in enumerate(v['rotor_axes']):
        assert values[f'CA_ROTOR{i}_PX']==pytest.approx(axis[0]-com[0])
        assert values[f'CA_ROTOR{i}_PY']==pytest.approx(com[1]-axis[1])
        assert values[f'CA_ROTOR{i}_PZ']==pytest.approx(com[2]-axis[2])
    assert values['CA_ROTOR0_KM']==.05  # motor-specific v1 parameters stay fixed


def test_live_session_is_authoritative_and_mixed_aircraft_is_rejected():
    live=SimpleNamespace(read_session=lambda:{'vehicle':'argo_v2','calibration_dir':'/tmp/selected-calibration'})
    with patch.object(profiles,'import_file',return_value=live),patch.dict(os.environ,{},clear=True):
        assert profiles.active_profile()['name']=='argo_v2'
        assert os.environ['PEGASUS_FASTLIVO_CONFIG_DIR']=='/tmp/selected-calibration'
        with pytest.raises(ValueError,match='Simulator is argo_v2'):
            profiles.active_profile('argo_v1')
