#!/usr/bin/env python3
"""Estimate one rigid gravity refinement from measured planar floor/ceiling patches.

This never flattens individual floors, stairs, or the recorded trajectory.
The rotation is only proposed; preparation explicitly opts into its JSON.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from pcd_scene import read_pcd


def audit(points):
    ids=np.linspace(0,len(points)-1,min(30000,len(points)),dtype=int)
    distances,indices=cKDTree(points).query(points[ids],k=30,workers=4)
    patches=points[indices].astype(float)
    centered=patches-patches.mean(axis=1,keepdims=True)
    values,vectors=np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered)/30.)
    normals=vectors[:,:,0];normals*=np.where(normals[:,2]<0.,-1.,1.)[:,None]
    good=(normals[:,2]>.98)&(values[:,0]<.015**2)&(values[:,1]>.004)&(distances[:,-1]<.65)
    normal=normals[good]
    if len(normal)<150:raise ValueError('Insufficient measured horizontal planar patches')
    slopes=-normal[:,:2]/normal[:,2,None]
    median=np.median(slopes,axis=0)
    consistent=np.linalg.norm(slopes-median,axis=1)<.035
    if consistent.mean()<.5:raise ValueError('Horizontal patches do not support one coherent global rotation')
    gradient=np.median(slopes[consistent],axis=0)
    n=np.r_[-gradient,1.];n/=np.linalg.norm(n)
    cross=np.cross(n,[0.,0.,1.]);x,y,z=cross
    skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    matrix=np.eye(3)+skew+skew@skew/(1.+n[2])
    angle=float(np.rad2deg(np.linalg.norm(Rotation.from_matrix(matrix).as_rotvec())))
    if angle>5.:raise ValueError('Proposed alignment exceeds 5 degrees; inspect reconstruction')
    return dict(method='One rigid rotation from robust measured horizontal local plane normals',
                matrix=matrix.tolist(),rotation_degrees=angle,source_floor_gradient_xy=gradient.tolist(),
                planar_patches=int(good.sum()),consistent_patches=int(consistent.sum()),
                slope_mad_xy=np.median(np.abs(slopes[consistent]-gradient),axis=0).tolist(),
                nonrigid_deformation=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(read_pcd(args.pcd))
    args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
