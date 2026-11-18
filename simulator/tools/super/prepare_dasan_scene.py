#!/usr/bin/env python3
"""Prepare the measured Dasan map and aligned trajectory for physical USD export.

No route, synthetic obstacles or global floor are generated. A small support
pad at a checked free patch stabilizes the parked vehicle on sampled terrain.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from pcd_scene import read_pcd
from merge_dasan_fastlivo import write_pcd


def level_rotation(report):
    """One rigid world alignment from independently fitted long floor surfaces.

    Use only the slope along each long corridor: the narrow transverse extent
    makes its other coefficient less reliable. No per-floor warping is applied.
    """
    slopes=[];support=[]
    for axis,names in ((0,('north','south')),(1,('west','east'))):
        values=[];weights=[]
        for name in names:
            for face in report[name]['floors']:
                minimum_support=max(180,.02*min(report[name]['points'],12000))
                if face['samples']<minimum_support or face['plane_residual_rms_m']>.04:continue
                values.append(face['coefficients'][axis])
                weights.append(face['samples']*face['along_corridor_length_m'])
        if len(values)<3:raise ValueError('Insufficient independent long surfaces for rigid leveling')
        order=np.argsort(values);v=np.array(values)[order];w=np.array(weights)[order]
        slopes.append(float(v[np.searchsorted(np.cumsum(w),w.sum()/2)]));support.append(len(v))
    normal=np.array([-slopes[0],-slopes[1],1.]);normal/=np.linalg.norm(normal)
    target=np.array([0.,0.,1.]);cross=np.cross(normal,target)
    skew=np.array([[0.,-cross[2],cross[1]],[cross[2],0.,-cross[0]],[-cross[1],cross[0],0.]])
    matrix=np.eye(3)+skew+skew@skew/(1.+normal@target)
    diagnostics=dict(method='Single rigid rotation from median long-corridor surface slopes',
        source_floor_gradient_xy=slopes,independent_surface_counts_xy=support,
        rotation_degrees=float(np.degrees(np.linalg.norm(Rotation.from_matrix(matrix).as_rotvec()))),
        matrix=matrix.tolist(),nonrigid_deformation=False)
    return matrix,diagnostics


def choose_start(points, initial):
    """Choose a supported, clear low-level patch near the first measured pose."""
    # A carried sensor starts above the initial floor. Restrict the search to
    # that level; higher floors are not landing-pad candidates.
    local=points[np.linalg.norm(points[:,:2]-initial[:2],axis=1)<7.]
    floor=local[(local[:,2]>initial[2]-1.5)&(local[:,2]<initial[2]+.15)]
    if len(floor)<50:raise ValueError('Insufficient observed initial floor')
    tree=cKDTree(floor[:,:2]);candidates=[]
    for x in np.arange(initial[0]-4.,initial[0]+4.01,.25):
        for y in np.arange(initial[1]-4.,initial[1]+4.01,.25):
            if np.linalg.norm([x-initial[0],y-initial[1]])>5.:continue
            patch=floor[tree.query_ball_point([x,y],.75)]
            if len(patch)<35:continue
            # Revisited floor samples may have lower duplicate layers from
            # odometry drift. Require a supported flat upper stratum, and place
            # the pad above ALL nearby samples rather than deleting layers.
            high=float(np.percentile(patch[:,2],90))
            upper=patch[(patch[:,2]>high-.10)&(patch[:,2]<high+.05)]
            if len(upper)<20:continue
            low,high=np.percentile(upper[:,2],[10,90])
            if high-low>.10:continue
            ground_patch=patch
            z=float(np.ceil(ground_patch[:,2].max()/.1)*.1+.02)
            column=local[(local[:,2]>z+.18)&(local[:,2]<z+2.)]
            distance=np.linalg.norm(column[:,:2]-[x,y],axis=1).min()if len(column)else 10.
            if distance<1.:continue
            score=min(float(distance),2.)-.12*np.linalg.norm([x-initial[0],y-initial[1]])
            candidates.append((score,np.array([x,y,z])))
    if not candidates:raise ValueError('No clear supported start patch near the initial pose')
    return max(candidates,key=lambda v:v[0])[1]


def prepare(mapping, assets, level_audit=None):
    assets.mkdir(parents=True, exist_ok=True)
    raw_path = mapping/'dasan_raw.pcd'
    raw = read_pcd(raw_path)
    metadata_path=mapping/'reconstruction.json'
    metadata=json.loads(metadata_path.read_text())if metadata_path.exists()else {'backend':'Super-LIO'}
    backend=metadata['backend']
    distances = cKDTree(raw).query(raw, k=5, workers=4)[0][:,-1]
    points = raw[distances < .4]
    trajectory = np.genfromtxt(mapping/'trajectory.csv', delimiter=',', names=True)
    columns = np.column_stack([trajectory[k] for k in ('t','x','y','z','qx','qy','qz','qw')])
    alignment=np.eye(3);leveling=None
    if level_audit:
        alignment,leveling=level_rotation(json.loads(Path(level_audit).read_text()))
        points=(points@alignment.T).astype(np.float32)
        columns[:,1:4]=columns[:,1:4]@alignment.T
        columns[:,4:8]=(Rotation.from_matrix(alignment)*Rotation.from_quat(columns[:,4:8])).as_quat()
    origin = choose_start(points,columns[0,1:4])if backend=='FAST-LIVO2'else np.array([1.5,3.75,0.])
    points = (points-origin).astype(np.float32)
    voxel_lower = np.floor(points.astype(np.float64)/.1)*.1
    vertical = (voxel_lower[:,2]+.1 > .2) & (voxel_lower[:,2] < 2.)
    boxes = voxel_lower[vertical]
    nearest_xy = np.maximum(np.maximum(boxes[:,:2], -(boxes[:,:2]+.1)),0.)
    clearance = float(np.linalg.norm(nearest_xy,axis=1).min())
    if clearance < .7:
        raise ValueError(f'Start patch clearance too small: {clearance}')
    ground = points[(np.linalg.norm(points[:,:2],axis=1)<.7)&(points[:,2]<.2)]
    if len(ground)<20 or ground[:,2].max()>.01:
        raise ValueError('Start support patch does not match the measured ground')
    path = assets/'dasan.pcd'
    with path.open('wb') as f:
        f.write((f'# Dasan {backend} reconstruction; metres, Z-up\nVERSION .7\n'
                 'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                 f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                 f'POINTS {len(points)}\nDATA binary\n').encode())
        points.astype('<f4').tofile(f)
    columns[:,1:4] -= origin
    np.savetxt(assets/'dasan_trajectory.csv', columns, delimiter=',',
               header='t,x,y,z,qx,qy,qz,qw', comments='')
    recipe = dict(
        schema_version=1, source_bag=str(Path.home()/'Bags/Dasan'),
        source_pcd=str(raw_path.resolve()),
        source_pcd_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),
        generated_pcd_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        mapping_config=str((mapping/'mapping.yaml').resolve()),
        reconstruction=f'{backend}; no loop closure or external ground truth',
        backend=backend, reconstruction_metadata=metadata,
        sensor_translation_m=[-.011,-.02329,-.04412],
        input_points=len(raw), points=len(points),
        filtering=dict(method='radius support', radius_m=.4, minimum_neighbors_including_self=5,
                       removed_points=len(raw)-len(points)),
        source_origin_in_world_m=(-origin).tolist(),
        world_from_lio_translation_m=(-origin).tolist(),
        world_from_lio_rotation=alignment.tolist(),rigid_level_alignment=leveling,
        spawn=[0.,0.,.10], spawn_yaw=float(np.pi/2),
        landing_pad=dict(center=[0.,0.,0.], half_size=.5),
        start_column_voxel_xy_clearance_m=clearance,
        start_ground_z_range_m=[float(ground[:,2].min()),float(ground[:,2].max())],
        pad_note='One 1x1 m support plane above measured local ground; no obstacles removed',
        trajectory=dict(file='dasan_trajectory.csv',poses=len(columns),
                        duration_s=float(columns[-1,0]-columns[0,0]),
                        length_m=float(np.linalg.norm(np.diff(columns[:,1:4],axis=0),axis=1).sum())),
        bounds_min=points.min(0).tolist(),bounds_max=points.max(0).tolist(),
        limitations=['Unobserved surfaces are not reconstructed.',
                     'Moving objects and accumulated LIO drift may remain.',
                     '10 cm surface voxels approximate measurements; sampling gaps may remain.'])
    (assets/'dasan_recipe.json').write_text(json.dumps(recipe,indent=2)+'\n')
    shutil.copyfile(mapping/'mapping.yaml',assets/'dasan_mapping.yaml')
    if (mapping/'native_color.npz').exists():
        native=np.load(mapping/'native_color.npz');keep=distances<.4
        colors=native['rgb'][keep];direct=native['direct'][keep];filled=native['filled'][keep]
        np.savez_compressed(assets/'dasan_color.npz',rgb=colors,direct=direct,filled=filled,
                            geometry_sha256=recipe['generated_pcd_sha256'])
        write_pcd(assets/'dasan_rgb.pcd',points,colors)
        color_metadata=dict(method='FAST-LIVO2 native RGB world points; same reconstruction as geometry',
            geometry_sha256=recipe['generated_pcd_sha256'],points=len(points),
            direct_camera_points=int(direct.sum()),small_gap_interpolated_points=int(filled.sum()),
            uncoloured_points=int((~(direct|filled)).sum()),
            rgb_transfer_max_distance_m=metadata['rgb_transfer_max_distance_m'],
            color_sha256=hashlib.sha256((assets/'dasan_color.npz').read_bytes()).hexdigest(),
            camera_calibration_file=str((mapping/'camera.yaml').resolve()))
        (assets/'dasan_color.json').write_text(json.dumps(color_metadata,indent=2)+'\n')
    elif (assets/'dasan_color.npz').exists():
        # Never silently retain colours from a different geometry.
        raise ValueError('Existing colour assets must be regenerated for this reconstruction')
    print(json.dumps(recipe,indent=2))
    return path


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mapping',type=Path)
    parser.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[2]/'assets/super')
    parser.add_argument('--level-audit',type=Path,help='Optional independent corridor audit for ONE rigid world rotation')
    args=parser.parse_args()
    prepare(args.mapping,args.assets,args.level_audit)
