#!/usr/bin/env python3
"""Cross-check measured map, native color, physical export and mission assets."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/super'))
from pcd_scene import read_pcd,scene_options


def check(directory,scenario):
    pcd=directory/f'{scenario}.pcd';cloud=read_pcd(pcd)
    assert len(cloud)>100 and np.isfinite(cloud).all()
    digest=hashlib.sha256(pcd.read_bytes()).hexdigest()
    recipe=json.loads((directory/f'{scenario}_recipe.json').read_text())
    mission=json.loads((directory/f'{scenario}_mission.json').read_text())
    export=json.loads((directory/f'{scenario}.json').read_text())
    assert recipe['generated_pcd_sha256']==mission['map_sha256']==export['source_sha256']==digest
    assert np.allclose(cloud.min(0),export['bounds_min']) and np.allclose(cloud.max(0),export['bounds_max'])
    assert not export['flat_floor'] and export['exposed_faces']>0 and export['chunks']>0
    assert export['landing_pad']==mission['landing_pad']==scene_options(directory,scenario)['landing_pad']
    color=np.load(directory/f'{scenario}_color.npz')
    assert str(color['geometry_sha256'])==digest and color['rgb'].shape==(len(cloud),3)
    assert color['direct'].shape==(len(cloud),) and color['filled'].shape==(len(cloud),)
    assert hashlib.sha256((directory/f'{scenario}_color.npz').read_bytes()).hexdigest()==export['colors_sha256']
    path=np.array([mission['takeoff'],*mission['waypoints']])
    assert np.isfinite(path).all()
    if mission.get('return_to_start',True):assert np.allclose(path[0],path[-1])
    if mission.get('ascent_only',False):
        assert path[-1,2]>path[0,2]+10. and not mission['return_to_start']
    assert len(mission['yaw_waypoints'])==len(path)
    for yaw in mission['yaw_waypoints']:assert yaw is None or np.isfinite(yaw)
    samples=np.concatenate([np.linspace(a,b,max(2,int(np.ceil(np.linalg.norm(b-a)/.05))+1))for a,b in zip(path[:-1],path[1:])])
    centers=np.unique(np.floor(cloud.astype(float)/.1).astype(np.int32),axis=0)*.1+.05
    clearance=cKDTree(centers).query(samples,workers=4)[0]-np.sqrt(3)*.05
    assert clearance.min()>=mission['offline_clearance_m']-1e-5
    return dict(scenario=scenario,points=len(cloud),map_sha256=digest,route_length_m=mission['length_m'],
                minimum_dense_center_clearance_m=float(clearance.min()),native_rgb_points=int(color['direct'].sum()))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario',choices=('dasan','frontier'))
    parser.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[1]/'assets/super')
    args=parser.parse_args();print(json.dumps(check(args.assets,args.scenario),indent=2))
