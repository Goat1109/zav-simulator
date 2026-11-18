#!/usr/bin/env python3
"""Small USD regression: binary PCD, negative cells, culling and camera RGB.

Run with Isaac's Python and omni.usd.libs on PYTHONPATH/LD_LIBRARY_PATH.
"""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/super'))
from pcd_scene import read_pcd, build_scene


with tempfile.TemporaryDirectory() as directory:
    directory=Path(directory)
    source=directory/'binary.pcd'
    xyz=np.array([[-.09,-.09,.11],[.01,-.09,.11]],dtype=np.float32)
    records=np.zeros(2,dtype=[('rgb','<u4'),('z','<f4'),('y','<f4'),('x','<f4')])
    for i,key in enumerate(('x','y','z')): records[key]=xyz[:,i]
    with source.open('wb')as file:
        file.write(b'VERSION .7\nFIELDS rgb z y x\nSIZE 4 4 4 4\nTYPE U F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n')
        records.tofile(file)
    np.testing.assert_array_equal(read_pcd(source),xyz)
    bad=directory/'truncated.pcd';bad.write_text('VERSION .7\n')
    try: read_pcd(bad)
    except ValueError: pass
    else: raise AssertionError('Truncated PCD must be rejected')
    plain=build_scene(source,directory/'plain.usdc',resolution=.1,flat_floor=False)
    assert json.loads(plain.with_suffix('.json').read_text())['exposed_faces']==10
    colors=directory/'color.npz'
    np.savez(colors,rgb=np.array([[255,0,0],[0,0,255]],dtype=np.uint8),
             direct=np.ones(2,dtype=bool),filled=np.zeros(2,dtype=bool),
             geometry_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    colored=build_scene(source,directory/'color.usdc',resolution=.1,flat_floor=False,colors_path=colors)
    a,b=Usd.Stage.Open(str(plain)),Usd.Stage.Open(str(colored))
    count=0;values=[]
    for prim in a.Traverse():
        if not prim.IsA(UsdGeom.Mesh):continue
        am=UsdGeom.Mesh(prim);bp=b.GetPrimAtPath(prim.GetPath());bm=UsdGeom.Mesh(bp)
        assert bp.HasAPI(UsdPhysics.CollisionAPI)
        for getter in ('GetPointsAttr','GetFaceVertexCountsAttr','GetFaceVertexIndicesAttr'):
            np.testing.assert_array_equal(np.asarray(getattr(am,getter)().Get()),np.asarray(getattr(bm,getter)().Get()))
        pv=bm.GetDisplayColorPrimvar();assert pv.GetInterpolation()=='uniform'
        value=np.asarray(pv.Get());assert len(value)==len(bm.GetFaceVertexCountsAttr().Get())
        values.extend(value);count+=len(value)
    assert count==10
    values=np.array(values)
    assert np.count_nonzero(np.all(values==[1,0,0],axis=1))==5
    assert np.count_nonzero(np.all(values==[0,0,1],axis=1))==5
    before=colored.stat().st_mtime_ns
    build_scene(source,colored,resolution=.1,flat_floor=False,colors_path=colors)
    assert colored.stat().st_mtime_ns==before
print('PCD_SCENE_OK: binary fields, truncated header, adjacent negative voxels, RGB, unchanged collision geometry, cache')
