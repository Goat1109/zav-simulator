"""Write the complete numerical aircraft-registration audit (USD-enabled Python)."""
from pathlib import Path
import hashlib
import json
import sys
import numpy as np
import yaml
from pxr import Usd,UsdGeom,UsdPhysics
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from vehicle_profiles import SIMULATOR,load_profile,sensor_calibration,px4_overrides,total_com


def audit():
    v=load_profile('argo_v2');v1=load_profile('argo_v1')
    stage=Usd.Stage.Open(str(SIMULATOR/v['usd']));cad=Usd.Stage.Open(str(SIMULATOR/v['cad']))
    bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']);cache=UsdGeom.XformCache()
    com=np.array(total_com(v));inertia=np.zeros((3,3));principal={}
    for name,mass,position,tensor in [('body',v['body_mass'],v['body_com'],v['body_inertia']),
            *[(f'rotor{i}',v['rotor_mass'],v['rotor_coms'][i],v['rotor_inertia'])for i in range(4)]]:
        delta=np.asarray(position)-com
        inertia+=np.asarray(tensor)+mass*((delta@delta)*np.eye(3)-np.outer(delta,delta))
        api=UsdPhysics.MassAPI(stage.GetPrimAtPath('/vehicle/'+name));q=api.GetPrincipalAxesAttr().Get()
        principal[name]={'diagonal_kg_m2':list(api.GetDiagonalInertiaAttr().Get()),
                         'axes_quaternion_wxyz':[q.GetReal(),*q.GetImaginary()],
                         'local_com_m':list(api.GetCenterOfMassAttr().Get())}
    measurements={}
    for name in sorted(set(v['collision_meshes']+v['rotor_meshes'])):
        prim=cad.GetPrimAtPath('/Zenith_Argo_V2/'+name);box=bounds.ComputeWorldBound(prim).ComputeAlignedRange()
        measurements[name]={'bounds_mm':[list(box.GetMin()),list(box.GetMax())],
                            'transform_row_matrix_mm':[list(row)for row in cache.GetLocalToWorldTransform(prim)]}
    rootbox=bounds.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    report={'vehicle':'argo_v2','unit_factors':{'g_to_kg':.001,'mm_to_m':.001,'g_mm2_to_kg_m2':1e-9},
            'profile':v,'v1_comparison':v1,'motor_parameters_equal_to_v1':v['propulsion']==v1['propulsion'],
            'total_com_m':com.tolist(),'combined_inertia_about_total_com_at_zero_rotor_angles_kg_m2':inertia.tolist(),
            'authored_usd_principal_inertias':principal,
            'body_bounds_m':[list(rootbox.GetMin()),list(rootbox.GetMax())],
            'estimator_calibration':yaml.safe_load((SIMULATOR/v['calibration_dir']/'zenith.yaml').read_text()),
            'camera_calibration':yaml.safe_load((SIMULATOR/v['calibration_dir']/'zenith_camera_pinhole.yaml').read_text()),
            'sensor_mounts':sensor_calibration(v),'px4_overrides':px4_overrides(v),
            'cad_measurements':measurements,'files':{}}
    for rel in [v['cad'],v['usd'],'config/vehicles/argo_v2.json',
                v['calibration_dir']+'/zenith.yaml',v['calibration_dir']+'/zenith_camera_pinhole.yaml',
                'px4/10021_isaac_argo_v2']:
        file=SIMULATOR/rel
        report['files']['simulator/'+rel]={'sha256':hashlib.sha256(file.read_bytes()).hexdigest(),'bytes':file.stat().st_size}
    output=SIMULATOR/'config/vehicles/argo_v2_audit.json'
    output.write_text(json.dumps(report,indent=2)+'\n');print(output)
    return report


if __name__=='__main__':
    audit()
