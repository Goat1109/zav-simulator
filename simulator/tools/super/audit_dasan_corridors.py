#!/usr/bin/env python3
"""Measure parallel-floor slope and thickness in four long corridor strips.

This is a geometric diagnostic, not a transform or a map repair. Each surface
is fitted independently, so differing slopes expose distortion that one rigid
world alignment cannot remove. Regions are in the original LIO frame.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from pcd_scene import read_pcd


REGIONS={
    'west':([6.,-30.,-2.],[11.,30.,22.],1),
    'east':([44.,-30.,-2.],[50.,30.,22.],1),
    'south':([15.,-39.,-2.],[40.,-32.,22.],0),
    'north':([15.,33.,-2.],[40.,39.,22.],0),
}


def fit_planes(points,axis,seed=43):
    rng=np.random.default_rng(seed)
    if len(points)>12000:sample=points[rng.choice(len(points),12000,replace=False)].astype(float)
    else:sample=points.astype(float)
    result=[]
    for _ in range(7):
        if len(sample)<300:break
        X=np.c_[sample[:,:2],np.ones(len(sample))];best=None;best_count=0
        for _ in range(1000):
            anchor=int(rng.integers(len(sample)))
            near=np.flatnonzero(np.abs(sample[:,2]-sample[anchor,2])<.65)
            if len(near)<3:continue
            idx=rng.choice(near,3,replace=False)
            if abs(np.linalg.det(X[idx]))<.3:continue
            coef=np.linalg.solve(X[idx],sample[idx,2])
            if np.linalg.norm(coef[:2])>.2:continue
            error=np.abs(X@coef-sample[:,2]);inlier=error<.055
            count=int(inlier.sum())
            if count>best_count:best,best_count=coef,count
        if best is None or best_count<100:break
        mask=np.abs(X@best-sample[:,2])<.07
        for _ in range(3):
            coef=np.linalg.lstsq(X[mask],sample[mask,2],rcond=None)[0]
            mask=np.abs(X@coef-sample[:,2])<.07
        q=sample[mask];error=q[:,2]-np.c_[q[:,:2],np.ones(len(q))]@coef
        span=np.percentile(q[:,axis],[5,95]);length=float(span[1]-span[0])
        if length>15:
            centre=np.median(q[:,:2],axis=0)
            result.append(dict(coefficients=coef.tolist(),samples=len(q),
                slope_deg=float(np.degrees(np.arctan(np.linalg.norm(coef[:2])))),
                along_corridor_slope_deg=float(np.degrees(np.arctan(coef[axis]))),
                along_corridor_length_m=length,along_corridor_rise_m=float(coef[axis]*length),
                height_at_region_centre_m=float(centre@coef[:2]+coef[2]),
                plane_residual_rms_m=float(np.sqrt(np.mean(error**2)))))
        # Remove a wider band to avoid reporting the same noisy floor twice.
        sample=sample[np.abs(X@coef-sample[:,2])>.18]
    return sorted(result,key=lambda v:v['height_at_region_centre_m'])


def audit(points):
    report={}
    for name,(lower,upper,axis)in REGIONS.items():
        keep=((points>=lower)&(points<=upper)).all(1)
        report[name]={'points':int(keep.sum()),'floors':fit_planes(points[keep],axis)}
    return report


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('pcd',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();report=audit(read_pcd(args.pcd))
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    for name,region in report.items():
        print(name,[(round(f['height_at_region_centre_m'],2),round(f['along_corridor_slope_deg'],2),round(f['along_corridor_rise_m'],2))for f in region['floors']])
