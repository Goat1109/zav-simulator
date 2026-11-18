"""Execute the real physics callback with a stale, deliberately unavailable USD stage."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
from scipy.spatial.transform import Rotation
source=Path(__file__).resolve().parents[1]/'pegasus/simulator/logic/vehicles/vehicle.py'
tree=ast.parse(source.read_text())
vehicle=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Vehicle')
method=next(n for n in vehicle.body if isinstance(n,ast.FunctionDef) and n.name=='update_state')
module=ast.Module(body=[method],type_ignores=[]);env={'np':np,'Rotation':Rotation}
exec(compile(module,str(source),'exec'),env)
q=Rotation.from_euler('ZYX',[.6,-.2,.4]).as_quat()
w_body=np.array([.3,-.5,.8]);w_world=Rotation.from_quat(q).apply(w_body)
v0=np.array([1.,2.,3.]);v1=np.array([1.1,1.8,3.3])
pose=NS(p=[2.,3.,4.],r=NS(**dict(zip('xyzw',q))))
dc=NS(get_rigid_body=lambda name:42,get_rigid_body_pose=lambda handle:pose,
      get_rigid_body_angular_velocity=lambda handle:w_world,
      get_rigid_body_linear_velocity=lambda handle:v1)
state=NS(linear_velocity=v0)
instance=NS(_stage_prefix='/World/drone',get_dc_interface=lambda:dc,_state=state)
env['update_state'](instance,.005)
np.testing.assert_allclose(state.position,[2.,3.,4.])
np.testing.assert_allclose(state.attitude,q)
np.testing.assert_allclose(state.angular_velocity,w_body,atol=1e-12)
np.testing.assert_allclose(state.linear_body_velocity,Rotation.from_quat(q).inv().apply(v1))
np.testing.assert_allclose(state.linear_acceleration,(v1-v0)/.005)
print('PASS: pose, world/body velocities and acceleration use one physics state without USD/render access')
