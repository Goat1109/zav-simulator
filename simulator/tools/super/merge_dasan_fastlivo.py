#!/usr/bin/env python3
"""Merge captured FAST-LIVO2 full scans and transfer its native camera RGB.

Streaming voxel centroids bound memory without changing any scan pose. The
result has no geometric leveling, floor flattening or trajectory deformation.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


def merge(paths,resolution,color=False):
    cells=[];totals=[];counts=[];rgb_totals=[]
    for index,path in enumerate(paths):
        data=np.load(path);xyz=data['xyz'].astype(np.float64)
        cell=np.floor(xyz/resolution).astype(np.int32)
        unique,inverse=np.unique(cell,axis=0,return_inverse=True)
        counts.append(np.bincount(inverse).astype(np.int64))
        cells.append(unique)
        totals.append(np.column_stack([np.bincount(inverse,weights=xyz[:,i])for i in range(3)]))
        if color:
            rgb_totals.append(np.column_stack([np.bincount(inverse,weights=data['rgb'][:,i])for i in range(3)]))
        if index%10==0:print(f'[Merge] compacted {index+1}/{len(paths)} fragments',flush=True)
    if not cells:raise ValueError('No captured fragments')
    _,inverse=np.unique(np.concatenate(cells),axis=0,return_inverse=True)
    count=np.bincount(inverse,weights=np.concatenate(counts))
    total=np.concatenate(totals)
    sums=np.column_stack([np.bincount(inverse,weights=total[:,i])for i in range(3)])
    rgb=None
    if color:
        rgb_total=np.concatenate(rgb_totals)
        summed=np.column_stack([np.bincount(inverse,weights=rgb_total[:,i])for i in range(3)])
        rgb=np.rint(summed/count[:,None]).astype(np.uint8)
    return (sums/count[:,None]).astype(np.float32),rgb


def write_pcd(path,xyz,rgb=None):
    if rgb is None:
        data=xyz.astype('<f4');fields='x y z';size='4 4 4';kind='F F F';count='1 1 1'
    else:
        data=np.empty(len(xyz),dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
        for i,k in enumerate(('x','y','z')):data[k]=xyz[:,i]
        data['rgb']=(rgb[:,0].astype(np.uint32)<<16)|(rgb[:,1].astype(np.uint32)<<8)|rgb[:,2].astype(np.uint32)
        fields='x y z rgb';size='4 4 4 4';kind='F F F U';count='1 1 1 1'
    with Path(path).open('wb')as f:
        f.write((f'VERSION .7\nFIELDS {fields}\nSIZE {size}\nTYPE {kind}\nCOUNT {count}\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n').encode())
        data.tofile(f)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mapping',type=Path)
    parser.add_argument('--name',choices=('dasan','frontier'),default='dasan')
    parser.add_argument('--resolution',type=float,default=.1)
    args=parser.parse_args();root=args.mapping
    capture=json.loads((root/'capture.json').read_text())
    if not capture['complete']:raise ValueError('Replay did not complete')
    xyz,_=merge(sorted((root/'fragments').glob('full_*.npz')),args.resolution)
    coloured,rgb=merge(sorted((root/'fragments').glob('rgb_*.npz')),args.resolution,True)
    write_pcd(root/f'{args.name}_raw.pcd',xyz)
    write_pcd(root/'native_rgb.pcd',coloured,rgb)
    distance,nearest=cKDTree(coloured).query(xyz,distance_upper_bound=.15,workers=4)
    direct=np.isfinite(distance);colors=np.full((len(xyz),3),128,dtype=np.uint8)
    colors[direct]=rgb[nearest[direct]]
    np.savez_compressed(root/'native_color.npz',rgb=colors,direct=direct,filled=np.zeros(len(xyz),bool))
    metadata=dict(backend='FAST-LIVO2',method='Native LIVO full world scans with camera updates',
                  voxel_centroid_resolution_m=args.resolution,full_points=len(xyz),native_rgb_points=len(coloured),
                  rgb_transfer_max_distance_m=.15,direct_color_points=int(direct.sum()),
                  geometric_deformation_applied=False,capture=capture)
    (root/'reconstruction.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(metadata,indent=2))


if __name__=='__main__':main()
