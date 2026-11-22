"""Reproducibly author argo_v2 physics and moving propellers around preserved CAD.

Requires pxr and scipy in an Isaac/USD-enabled interpreter.
"""
from pathlib import Path
import sys
import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from vehicle_profiles import SIMULATOR, load_profile


def author_mass(prim, mass, com, inertia):
    inertia = np.asarray(inertia,dtype=float)
    if not np.allclose(inertia,inertia.T,atol=1e-15,rtol=0):
        raise ValueError('Inertia tensor is not symmetric')
    values,axes = np.linalg.eigh(inertia)
    if np.any(values<=0) or values[-1]>sum(values[:2])+1e-12:
        raise ValueError('Nonphysical inertia tensor')
    if np.linalg.det(axes)<0:
        axes[:,0]*=-1
    x,y,z,w = Rotation.from_matrix(axes).as_quat()
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(mass)
    api.CreateCenterOfMassAttr(Gf.Vec3f(*map(float,com)))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float,values)))
    api.CreatePrincipalAxesAttr(Gf.Quatf(float(w),Gf.Vec3f(float(x),float(y),float(z))))


def build():
    v=load_profile('argo_v2');source=SIMULATOR/v['cad'];output=SIMULATOR/v['usd']
    cad=Usd.Stage.Open(str(source));flat=cad.Flatten();root_path=cad.GetDefaultPrim().GetPath()
    stage=Usd.Stage.CreateInMemory();UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,'Z')
    root=UsdGeom.Xform.Define(stage,'/vehicle').GetPrim();stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    root.AddAppliedSchema('PhysxArticulationAPI')
    root.CreateAttribute('physxArticulation:enabledSelfCollisions',Sdf.ValueTypeNames.Bool).Set(False)
    root.SetCustomDataByKey('argo:vehicle','argo_v2')
    body=UsdGeom.Xform.Define(stage,'/vehicle/body').GetPrim();UsdPhysics.RigidBodyAPI.Apply(body)
    author_mass(body,v['body_mass'],v['body_com'],v['body_inertia'])
    visual=UsdGeom.Xform.Define(stage,'/vehicle/body/visual_mesh')
    visual.GetPrim().GetReferences().AddReference(str(source))
    visual.AddScaleOp().Set(Gf.Vec3f(v['cad_scale']))
    for name in v['collision_meshes']:
        part=stage.GetPrimAtPath(str(visual.GetPath())+'/'+name)
        if not part:
            raise ValueError(f'Missing collider CAD part: {name}')
        for prim in Usd.PrimRange(part):
            if not prim.IsA(UsdGeom.Mesh):continue
            UsdPhysics.CollisionAPI.Apply(prim)
            approximation='convexDecomposition' if name in v['decomposition_meshes'] else 'convexHull'
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
            prim.AddAppliedSchema('PhysxCollisionAPI')
            prim.CreateAttribute('physxCollision:contactOffset',Sdf.ValueTypeNames.Float).Set(.002)
            prim.CreateAttribute('physxCollision:restOffset',Sdf.ValueTypeNames.Float).Set(0.)
    cache=UsdGeom.XformCache()
    for i,(position,com,name) in enumerate(zip(v['rotor_axes'],v['rotor_coms'],v['rotor_meshes'])):
        path=f'/vehicle/rotor{i}';rotor=UsdGeom.Xform.Define(stage,path)
        rotor.AddTranslateOp().Set(Gf.Vec3d(*position));UsdPhysics.RigidBodyAPI.Apply(rotor.GetPrim())
        author_mass(rotor.GetPrim(),v['rotor_mass'],np.array(com)-position,v['rotor_inertia'])
        source_part=cad.GetPrimAtPath(str(root_path)+'/'+name)
        target=path+'/visual_mesh'
        Sdf.CopySpec(flat,source_part.GetPath(),stage.GetRootLayer(),target)
        # Source units are mm; cancel the rotor's metre translation in the child.
        matrix=cache.GetLocalToWorldTransform(source_part)*Gf.Matrix4d().SetScale(v['cad_scale'])
        matrix=matrix*Gf.Matrix4d().SetTranslate(Gf.Vec3d(*[-x for x in position]))
        mesh=UsdGeom.Xformable(stage.GetPrimAtPath(target));mesh.ClearXformOpOrder();mesh.AddTransformOp().Set(matrix)
        for prim in Usd.PrimRange(stage.GetPrimAtPath(target)):
            for rel in prim.GetRelationships():
                rel.SetTargets([t.ReplacePrefix(root_path,visual.GetPath()) for t in rel.GetTargets()])
        UsdGeom.Imageable(stage.GetPrimAtPath(str(visual.GetPath())+'/'+name)).CreateVisibilityAttr('invisible')
        joint=UsdPhysics.RevoluteJoint.Define(stage,path+f'/joint{i}')
        joint.CreateBody0Rel().SetTargets([body.GetPath()]);joint.CreateBody1Rel().SetTargets([rotor.GetPath()])
        joint.CreateAxisAttr('Z');joint.CreateLocalPos0Attr(Gf.Vec3f(*position));joint.CreateLocalPos1Attr(Gf.Vec3f(0.))
    stage.GetRootLayer().Export(str(output))
    output.write_text(output.read_text().replace('@'+str(source)+'@','@./'+source.name+'@').rstrip()+'\n')
    print(output)
    return output


if __name__=='__main__':
    build()
