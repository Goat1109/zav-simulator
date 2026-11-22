"""Verify authored SI tensors, CAD placement and independently supplied COMs."""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from vehicle_profiles import load_profile,SIMULATOR

v=load_profile('argo_v2');stage=Usd.Stage.Open(str(SIMULATOR/v['usd']))
cad=Usd.Stage.Open(str(SIMULATOR/v['cad']));cache=UsdGeom.XformCache();bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render'])
assert UsdGeom.GetStageMetersPerUnit(stage)==1.
assert stage.GetDefaultPrim().HasAPI(UsdPhysics.ArticulationRootAPI)
total=0.
for name,expected,inertia,com in [('body',v['body_mass'],v['body_inertia'],v['body_com']),
        *[(f'rotor{i}',v['rotor_mass'],v['rotor_inertia'],np.array(v['rotor_coms'][i])-v['rotor_axes'][i])for i in range(4)]]:
    prim=stage.GetPrimAtPath('/vehicle/'+name);api=UsdPhysics.MassAPI(prim)
    assert prim.HasAPI(UsdPhysics.RigidBodyAPI)
    assert abs(api.GetMassAttr().Get()-expected)<1e-6
    total+=api.GetMassAttr().Get()
    q=api.GetPrincipalAxesAttr().Get();r=Rotation.from_quat([*q.GetImaginary(),q.GetReal()]).as_matrix()
    tensor=r@np.diag(api.GetDiagonalInertiaAttr().Get())@r.T
    assert np.allclose(tensor,inertia,atol=3e-9,rtol=1e-6),(name,tensor)
    assert np.allclose(api.GetCenterOfMassAttr().Get(),com,atol=3e-9,rtol=0)
assert abs(total-v['mass'])<1e-6
for i,name in enumerate(v['rotor_meshes']):
    original=bounds.ComputeWorldBound(cad.GetPrimAtPath('/Zenith_Argo_V2/'+name)).ComputeAlignedRange()
    moved=bounds.ComputeWorldBound(stage.GetPrimAtPath(f'/vehicle/rotor{i}/visual_mesh')).ComputeAlignedRange()
    assert np.allclose(np.array(moved.GetMin()),np.array(original.GetMin())*.001,atol=2e-8)
    assert np.allclose(np.array(moved.GetMax()),np.array(original.GetMax())*.001,atol=2e-8)
    assert UsdPhysics.RevoluteJoint(stage.GetPrimAtPath(f'/vehicle/rotor{i}/joint{i}'))
    assert UsdGeom.Imageable(stage.GetPrimAtPath('/vehicle/body/visual_mesh/'+name)).ComputeVisibility()=='invisible'
colliders=[p for p in stage.Traverse()if p.HasAPI(UsdPhysics.CollisionAPI)]
assert len(colliders)==len(v['collision_meshes'])
assert all(UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() for p in colliders)
print(f'ARGO_V2_USD_OK: 5 rigid bodies, 4 joints, {len(colliders)} colliders; SI inertia/COM/CAD transforms verified')
