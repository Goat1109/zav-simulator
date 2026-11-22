"""Isaac headless check: five authored masses, four joints, balanced 2 s thrust.

No PX4, ROS, estimator or feedback control; validates physical asset authoring.
"""
from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from pathlib import Path
import sys
import json
import numpy as np
from scipy.spatial.transform import Rotation
from isaacsim.core.api import World
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vehicle_profiles import load_profile,total_com
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import MultirotorConfig
from pegasus.simulator.logic.vehicles.multirotors import ArgoV2
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve


class Hover:
    def initialize(self,vehicle): self.vehicle=vehicle
    def input_reference(self): return self.speeds
    def start(self): pass
    def stop(self): pass
    def reset(self): pass
    def update(self,dt): pass
    def update_state(self,state): pass
    def update_sensor(self,*args): pass
    def update_graphical_sensor(self,*args): pass


try:
    v=load_profile('argo_v2');p=v['propulsion'];com=np.array(total_com(v));axes=np.array(v['rotor_axes'])-com
    allocation=np.stack([np.ones(4),axes[:,1],-axes[:,0],
                         np.array(p['rot_dir'])*p['rolling_moment_coefficient']/p['rotor_constant']])
    forces=np.linalg.solve(allocation,[v['mass']*9.81,0.,0.,0.])
    backend=Hover();backend.speeds=np.sqrt(forces/p['rotor_constant']).tolist()
    config=MultirotorConfig();config.sensors=[];config.backends=[backend]
    config.thrust_curve=QuadraticThrustCurve({
        'rotor_constant':[p['rotor_constant']]*4,'rolling_moment_coefficient':[p['rolling_moment_coefficient']]*4,
        'rot_dir':p['rot_dir'],'max_rotor_velocity':[p['max_rotor_velocity']]*4})
    interface=PegasusInterface();world=World(physics_dt=.005,rendering_dt=.05)
    interface._world=world
    drone=ArgoV2(init_pos=[0.,0.,2.],init_orientation=[0.,0.,0.,1.],config=config)
    world.reset();world.play()
    dc=drone.get_dc_interface();art=dc.get_articulation(drone._stage_prefix)
    assert dc.get_articulation_body_count(art)==5
    masses=[]
    for suffix,expected in [('body',v['body_mass']),*[(f'rotor{i}',v['rotor_mass']) for i in range(4)]]:
        body=dc.get_rigid_body(drone._stage_prefix+'/'+suffix)
        mass=dc.get_rigid_body_properties(body).mass;masses.append(mass)
        assert abs(mass-expected)<1e-6,(suffix,mass,expected)
    assert dc.get_articulation_dof_count(art)==4
    # Articulation initialization can include one gravity-only physics tick.
    # Account for that initial velocity; a constant thrust test has no feedback.
    for _ in range(10): world.step(render=False)
    initial_position=np.array(drone.state.position);initial_velocity=np.array(drone.state.linear_velocity)
    for _ in range(400): world.step(render=False)
    drift=np.array(drone.state.position)-initial_position
    residual=drift-2.*initial_velocity
    velocity_change=np.array(drone.state.linear_velocity)-initial_velocity
    angle=Rotation.from_quat(drone.state.attitude).magnitude()
    result=dict(vehicle='argo_v2',masses_kg=masses,total_mass_kg=sum(masses),duration_s=2.,
                drift_m=drift.tolist(),initial_velocity_mps=initial_velocity.tolist(),
                residual_after_initial_velocity_m=residual.tolist(),velocity_change_mps=velocity_change.tolist(),
                attitude_error_rad=float(angle),motor_speeds_rad_s=backend.speeds)
    assert np.linalg.norm(residual)<.10,result
    assert abs(velocity_change[2])<.01,result
    assert angle<.03,result
    out=Path(__file__).resolve().parents[2]/'log/vehicle-validation/argo_v2_physics.json'
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(json.dumps(result,indent=2)+'\n')
    print('ARGO_V2_PHYSICS_OK '+json.dumps(result),flush=True)
except BaseException:
    import traceback
    traceback.print_exc()
    sys.stderr.flush()
    raise
finally:
    app.close()
