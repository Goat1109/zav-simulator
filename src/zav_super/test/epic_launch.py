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
profiles={'gym':('gym-epic-full-local.yaml',.3,3.0),'frontier_whole':('frontier_bounds.yaml',.3,9.5),
    'frontier_single_floor':('frontier-first-floor-sim.yaml',1.2,1.6),'forest':('forest-launch-local.yaml',1.2,1.8)}
for key,(name,low,high) in profiles.items():
    ctx.launch_configurations['epic_profile']=key
    nodes=[]
    with tempfile.TemporaryDirectory() as temp,patch.object(module,'Node',side_effect=lambda **kw:nodes.append(kw) or kw),patch.object(module,'get_package_share_directory',side_effect=lambda package:str(root/'src'/package)),patch.object(module.tempfile,'mkdtemp',return_value=temp):
        values=yaml.safe_load((root/'src/zav_super/config/common.yaml').read_text())
        values['rog_map']['virtual_ceil_height']=10.
        source=Path(temp)/'source.yaml';source.write_text(yaml.safe_dump(values))
        ctx.launch_configurations['common_config']=str(source)
        actions=module.setup(ctx)
    assert [n['executable'] for n in nodes]==['epic_frame_adapter','super_adapter','fsm_node','super_ommpc']
    params=nodes[-1]['parameters'][1]
    assert params['epic_frame_required'] and params['epic_enabled'] and params['epic_profile']==key
    assert params['epic_flight_min']==low and params['epic_flight_max']==high
    source=yaml.safe_load((root/'src/epic_ros2/config'/name).read_text())['/**']['ros__parameters']
    assert len(params['epic_bounds_lower'])==source['box_num']*3
    assert params['epic_bounds_lower'][:3]==source['box_0.down']
    assert nodes[-1]['parameters'][-1]['auto_takeoff'] is False
    assert nodes[0]['parameters'][0]['reference_yaw_deg']==(-180.0 if key=='frontier_single_floor' else 0.)
# Profile values must not be clipped by WORLD numbers before anchoring.
ctx.launch_configurations['common_config']=str(root/'src/zav_super/config/common.yaml')
for profile,(_,low,high) in profiles.items():
    ctx.launch_configurations['epic_profile']=profile;nodes=[]
    with tempfile.TemporaryDirectory() as temp,patch.object(module,'Node',side_effect=lambda **kw:nodes.append(kw) or kw),patch.object(module,'get_package_share_directory',side_effect=lambda package:str(root/'src'/package)),patch.object(module.tempfile,'mkdtemp',return_value=temp):
        module.setup(ctx)
        assert nodes[-1]['parameters'][1]['epic_flight_min']==low
        assert nodes[-1]['parameters'][1]['epic_flight_max']==high
        config=yaml.safe_load((root/'src/zav_super/config/common.yaml').read_text())
        planner=nodes[-2]['parameters'][0]
        params=nodes[-1]['parameters'][1]
        assert planner['flight_height_guard_enabled']
        assert planner['mission_envelope_enabled']
        assert planner['mission_flight_min']==low and planner['mission_flight_max']==high
        assert planner['mission_bounds_lower']==params['epic_bounds_lower']
        assert params['trajectory_max_vel']==(3. if profile=='gym' else 1.) and params['trajectory_max_acc']==1.5 and params['trajectory_max_jerk']==3.
        assert abs(planner['flight_ceiling']-((3.5 if profile in ('gym','forest') else config['rog_map']['virtual_ceil_height'])-.53))<1e-9
        assert abs(planner['flight_floor']-(config['rog_map']['virtual_ground_height']+.23))<1e-9
ctx.launch_configurations['epic_profile']='../../unsafe' 
with tempfile.TemporaryDirectory() as temp,patch.object(module,'get_package_share_directory',side_effect=lambda package:str(root/'src'/package)),patch.object(module.tempfile,'mkdtemp',return_value=temp):
    try:module.setup(ctx)
    except ValueError:pass
    else:raise AssertionError('unlisted profile accepted')
module=load(root/'src/epic_ros2/launch/hardware.launch.py');ctx=LaunchContext()
ctx.launch_configurations.update(config=str(root/'src/epic_ros2/config/gym-epic-full-local.yaml'),max_vel='3.0',max_acc='1.5',max_jerk='3.0',flight_min='.3',flight_max='1.5',world_floor='-.4',world_ceiling='1.2',height_margin='.03')
nodes=[]
with patch.object(module,'Node',side_effect=lambda **kw:nodes.append(kw) or kw):module.setup(ctx)
assert len(nodes)==1 and nodes[0]['executable']=='exploration_node'
p=nodes[0]['parameters'][-1]
assert p['use_sim_time'] is False and p['fsm.external_local_planner'] is True and p['MaxVelMag']==3.
assert p['exploration.planning_workers']==2
assert p['MaxAccMag']==1.5 and p['MaxJerkMag']==3.
assert p['fsm.hardware_height_guard'] and p['fsm.world_ceiling']==1.2 and p['fsm.height_margin']==.03
assert p['odometry_topic']=='/epic/profile_odom' and p['cloud_topic']=='/epic/profile_cloud'
assert ('/px4ctrl/takeoff_land','/epic/unused_takeoff_land') in nodes[0]['remappings']
print('PASS: four launch profiles, original bounds, hardware flight bands, manual takeoff, exploration-only composition')
