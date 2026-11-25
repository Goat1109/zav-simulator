#!/usr/bin/env python3
"""Validate launch composition without executing actions or starting ROS nodes."""
import importlib.util,tempfile
from pathlib import Path
from unittest.mock import patch
import yaml
from launch import LaunchContext

root=Path(__file__).resolve().parents[3]
def load(path):
    spec=importlib.util.spec_from_file_location('launch_under_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
module=load(root/'src/zav_super/launch/super_navigation.launch.py')
ctx=LaunchContext()
ctx.launch_configurations.update(estimator='external',execute='true',common_config=str(root/'src/zav_super/config/common.yaml'),
    hardware_config=str(root/'src/zav_real_control/config/hardware.yaml'),cloud_topic='',cloud_frame='',
    odom_topic='/zav/lio/odometry',distance='10.',auto_takeoff='false',takeoff_height='.4',start_driver='false',epic_enabled='true')
profiles={'gym':('gym-epic-full-local.yaml',.3,1.5),'frontier_whole':('frontier_bounds.yaml',.3,9.5),
    'frontier_single_floor':('frontier-first-floor-sim.yaml',1.2,1.9)}
for key,(name,low,high) in profiles.items():
    ctx.launch_configurations['epic_profile']=key
    nodes=[]
    with tempfile.TemporaryDirectory() as temp,patch.object(module,'Node',side_effect=lambda **kw:nodes.append(kw) or kw),patch.object(module,'get_package_share_directory',side_effect=lambda package:str(root/'src'/package)),patch.object(module.tempfile,'mkdtemp',return_value=temp):
        actions=module.setup(ctx)
    assert [n['executable'] for n in nodes]==['epic_frame_adapter','super_adapter','fsm_node','super_ommpc']
    params=nodes[-1]['parameters'][1]
    assert params['epic_frame_required'] and params['epic_enabled'] and params['epic_profile']==key
    assert params['epic_flight_min']==low and params['epic_flight_max']==high
    source=yaml.safe_load((root/'src/epic_ros2/config'/name).read_text())['/**']['ros__parameters']
    assert len(params['epic_bounds_lower'])==source['box_num']*3
    assert params['epic_bounds_lower'][:3]==source['box_0.down']
    assert nodes[-1]['parameters'][-1]['auto_takeoff'] is False
    assert nodes[0]['parameters'][0]['reference_yaw_deg']==(-172.3865 if key=='frontier_single_floor' else 0.)
ctx.launch_configurations['epic_profile']='../../unsafe'
with tempfile.TemporaryDirectory() as temp,patch.object(module,'get_package_share_directory',side_effect=lambda package:str(root/'src'/package)),patch.object(module.tempfile,'mkdtemp',return_value=temp):
    try:module.setup(ctx)
    except ValueError:pass
    else:raise AssertionError('unlisted profile accepted')
module=load(root/'src/epic_ros2/launch/hardware.launch.py');ctx=LaunchContext()
ctx.launch_configurations.update(config=str(root/'src/epic_ros2/config/gym-epic-full-local.yaml'),flight_min='.3',flight_max='1.5')
nodes=[]
with patch.object(module,'Node',side_effect=lambda **kw:nodes.append(kw) or kw):module.setup(ctx)
assert len(nodes)==1 and nodes[0]['executable']=='exploration_node'
p=nodes[0]['parameters'][-1]
assert p['use_sim_time'] is False and p['fsm.external_local_planner'] is True and p['MaxVelMag']==1.
assert p['odometry_topic']=='/epic/profile_odom' and p['cloud_topic']=='/epic/profile_cloud'
assert ('/px4ctrl/takeoff_land','/epic/unused_takeoff_land') in nodes[0]['remappings']
print('PASS: three launch profiles, original bounds, hardware flight bands, manual takeoff, exploration-only composition')
