"""Turn upstream and generated mixed PCD surfaces into physical USD meshes.

No point cloud is injected into the planner. Each occupied 5 cm surface voxel
becomes geometry at its original map coordinate; only internal faces are culled.
The PCD is a sampled surface, so reconstruction has a documented voxel error.
"""
from pathlib import Path
import hashlib
import json
import os
import numpy as np

MAPS = {'high_speed': 'random_map_24_6635.pcd', 'dense': 'random_map_2_26609.pcd',
        'mixed': 'mixed_sparse_dense.pcd', 'dasan': 'dasan.pcd', 'frontier': 'frontier.pcd'}


def scene_pcd(directory, scenario):
    if scenario == 'mixed':
        try:
            from .mixed_scene import ensure_mixed_map
        except ImportError:
            from mixed_scene import ensure_mixed_map
        return ensure_mixed_map(directory)
    return Path(directory) / MAPS[scenario]


def read_pcd(path):
    with Path(path).open('rb') as f:
        header = {}
        while True:
            raw_line = f.readline()
            if not raw_line:
                raise ValueError('PCD ended before DATA header')
            line = raw_line.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key] = values
            if key == 'DATA':
                break
        if header['DATA'] == ['ascii']:
            data = np.loadtxt(f, dtype=np.float32, ndmin=2)
        elif header['DATA'] == ['binary']:
            kinds = {'F': 'f', 'I': 'i', 'U': 'u'}
            counts = list(map(int, header.get('COUNT', ['1']*len(header['FIELDS']))))
            dtype = np.dtype([(name, '<'+kinds[kind]+size, (count,))
                              for name, kind, size, count in zip(
                                  header['FIELDS'], header['TYPE'], header['SIZE'], counts)])
            records = np.fromfile(f, dtype=dtype)
            points = np.column_stack([records[k].reshape(-1) for k in ('x','y','z')])
            if len(points) != int(header['POINTS'][0]):
                raise ValueError('PCD binary payload length does not match header')
            return points[np.isfinite(points).all(axis=1)]
        else:
            raise ValueError('Supported PCD encodings: ascii, binary')
    fields = header['FIELDS']
    points = data[:, [fields.index(k) for k in ('x', 'y', 'z')]]
    if len(points) != int(header['POINTS'][0]):
        raise ValueError('PCD point count or coordinates invalid')
    return points[np.isfinite(points).all(axis=1)]


def surface_voxels(points, resolution):
    if not 0 < resolution <= .1:
        raise ValueError('Surface resolution must be in (0, 0.1] metres')
    return np.unique(np.floor(points.astype(np.float64) / resolution).astype(np.int32), axis=0)


def build_scene(pcd, output, resolution=.05, flat_floor=True, landing_pad=None, colors_path=None):
    from pxr import Usd, UsdGeom, UsdPhysics, Vt
    pcd, output = Path(pcd), Path(output)
    signature = {'source_sha256': hashlib.sha256(pcd.read_bytes()).hexdigest(),
                 'resolution_m': resolution, 'format_version': 3, 'flat_floor': flat_floor,
                 'landing_pad': landing_pad,
                 'colors_sha256': hashlib.sha256(Path(colors_path).read_bytes()).hexdigest()
                 if colors_path else None}
    manifest = output.with_suffix('.json')
    if output.exists() and manifest.exists() and all(
            json.loads(manifest.read_text()).get(k) == v for k, v in signature.items()):
        return output
    points = read_pcd(pcd)
    if not len(points):
        raise ValueError('Cannot build a scene from an empty point cloud')
    # The source contains a regular 10 cm sampling of a perfectly flat floor.
    # Reconstruct that plane exactly; isolated cubes would leave artificial
    # holes/steps and can destabilize the resting vehicle's IMU initialization.
    floor_mask = np.abs(points[:,2]) < 1e-7 if flat_floor else np.zeros(len(points), dtype=bool)
    floor_points = points[floor_mask]
    voxel_colors = None
    if colors_path:
        if not 0 < resolution <= .1:
            raise ValueError('Surface resolution must be in (0, 0.1] metres')
        colors = np.load(colors_path)
        if str(colors['geometry_sha256']) != signature['source_sha256']:
            raise ValueError('RGB and geometry hashes differ; regenerate camera projection')
        if colors['rgb'].shape != (len(points),3):
            raise ValueError('RGB point count differs from PCD')
        cells = np.floor(points[~floor_mask].astype(np.float64)/resolution).astype(np.int32)
        voxels, point_to_voxel = np.unique(cells,axis=0,return_inverse=True)
        valid=(colors['direct']|colors['filled'])[~floor_mask]
        weights=np.bincount(point_to_voxel,weights=valid,minlength=len(voxels))
        # USD displayColor is linear RGB; camera/PCD bytes remain original sRGB.
        srgb=colors['rgb'][~floor_mask].astype(np.float32)/255.
        linear=np.where(srgb<=.04045,srgb/12.92,((srgb+.055)/1.055)**2.4)
        voxel_colors=np.full((len(voxels),3),.21586,dtype=np.float32)
        for channel in range(3):
            summed=np.bincount(point_to_voxel,weights=linear[:,channel]*valid,minlength=len(voxels))
            voxel_colors[weights>0,channel]=summed[weights>0]/weights[weights>0]
    else:
        voxels = surface_voxels(points[~floor_mask], resolution)
    # Outward winding, one quad per exposed voxel face.
    faces = [((1,0,0), [(1,0,0),(1,1,0),(1,1,1),(1,0,1)]),
             ((-1,0,0), [(0,0,0),(0,0,1),(0,1,1),(0,1,0)]),
             ((0,1,0), [(0,1,0),(0,1,1),(1,1,1),(1,1,0)]),
             ((0,-1,0), [(0,0,0),(1,0,0),(1,0,1),(0,0,1)]),
             ((0,0,1), [(0,0,1),(1,0,1),(1,1,1),(0,1,1)]),
             ((0,0,-1), [(0,0,0),(0,1,0),(1,1,0),(1,0,0)])]
    # Packed integer lookup and one chunk at a time keep measured maps bounded
    # in memory; millions of Python tuples/float vertices otherwise exhaust RAM.
    lower_key = voxels.min(axis=0).astype(np.int64)-2
    shape_key = voxels.max(axis=0).astype(np.int64)-lower_key+3
    stride = np.array([shape_key[1]*shape_key[2],shape_key[2],1],dtype=np.int64)
    def keys(values): return (values.astype(np.int64)-lower_key) @ stride
    all_keys = np.sort(keys(voxels))
    chunk_coords, inverse = np.unique(voxels//160, axis=0, return_inverse=True)
    order = np.argsort(inverse, kind='stable')
    starts = np.r_[0,np.cumsum(np.bincount(inverse))]
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(output.stem + '.tmp.usdc')
    stage = Usd.Stage.CreateNew(str(temporary))
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, '/SUPER').GetPrim()
    stage.SetDefaultPrim(root)
    if len(floor_points) or landing_pad:
        if landing_pad:
            x,y,z = landing_pad['center']
            radius = landing_pad['half_size']
            lower=np.array([x-radius,y-radius,z]);upper=np.array([x+radius,y+radius,z])
        else:
            lower=floor_points.min(axis=0);upper=floor_points.max(axis=0)
        floor_z = float(lower[2])
        floor=UsdGeom.Mesh.Define(stage, '/SUPER/floor')
        floor.CreatePointsAttr([(float(lower[0]),float(lower[1]),floor_z),
                                (float(upper[0]),float(lower[1]),floor_z),
                                (float(upper[0]),float(upper[1]),floor_z),
                                (float(lower[0]),float(upper[1]),floor_z)])
        floor.CreateFaceVertexCountsAttr([4]);floor.CreateFaceVertexIndicesAttr([0,1,2,3])
        floor.CreateSubdivisionSchemeAttr('none');floor.CreateDoubleSidedAttr(True)
        floor.CreateDisplayColorAttr([(.25,.28,.3)])
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(floor.GetPrim()).CreateApproximationAttr('none')
    total_faces = 0
    for i in range(len(chunk_coords)):
        indices = order[starts[i]:starts[i+1]]
        cells = voxels[indices]
        pieces = []
        face_colors = []
        for direction,corners in faces:
            neighbor_keys = keys(cells+np.array(direction,dtype=np.int32))
            locations = np.searchsorted(all_keys, neighbor_keys)
            present = all_keys[np.minimum(locations,len(all_keys)-1)] == neighbor_keys
            exposed = cells[~present]
            if len(exposed):
                pieces.append(((exposed[:,None,:]+np.array(corners)[None,:,:])*resolution).reshape(-1,3))
                if voxel_colors is not None:
                    face_colors.append(voxel_colors[indices[~present]])
        if not pieces: continue
        values = np.concatenate(pieces).astype(np.float32)
        total_faces += len(values)//4
        mesh = UsdGeom.Mesh.Define(stage, f'/SUPER/surface_{i}')
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(values))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(values)//4,4,np.int32)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(values),dtype=np.int32)))
        mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(values))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        if voxel_colors is None:
            mesh.CreateDisplayColorAttr([(0.42,0.52,0.58)])
        else:
            mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform).Set(
                Vt.Vec3fArray.FromNumpy(np.concatenate(face_colors).astype(np.float32)))
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
    stage.GetRootLayer().Save()
    temporary.replace(output)
    signature.update(points=len(points), occupied_voxels=len(voxels), chunks=len(chunk_coords), exposed_faces=total_faces,
                     bounds_min=points.min(axis=0).tolist(), bounds_max=points.max(axis=0).tolist(),
                     max_surface_error_m=float(np.sqrt(3)*resolution))
    manifest.write_text(json.dumps(signature, indent=2)+'\n')
    return output


def scene_options(directory, scenario):
    """Identical hash-checked geometry settings for export and live simulation."""
    measured = scenario in ('dasan','frontier')
    pad = json.loads((directory/f'{scenario}_recipe.json').read_text())['landing_pad'] if measured else None
    if measured and (directory/f'{scenario}_mission.json').exists():
        mission=json.loads((directory/f'{scenario}_mission.json').read_text())
        if mission['map_sha256']!=hashlib.sha256((directory/f'{scenario}.pcd').read_bytes()).hexdigest():
            raise ValueError(f'{scenario} route and physical map hashes differ')
        pad=mission['landing_pad']
    colors_path = directory/f'{scenario}_color.npz' if measured and (directory/f'{scenario}_color.npz').exists() else None
    return dict(resolution=.1 if measured else .05,
                flat_floor=not measured, landing_pad=pad, colors_path=colors_path)


def add_super_scene(world, scenario):
    directory = Path(__file__).resolve().parents[2] / 'assets/super'
    options = scene_options(directory,scenario)
    override = os.environ.get('ARGO_SUPER_TERRAIN_USD')
    if override:
        from pxr import Usd, UsdGeom, UsdPhysics
        path = Path(override).expanduser().resolve()
        terrain = Usd.Stage.Open(str(path))
        if not terrain or not terrain.GetDefaultPrim():
            raise ValueError(f'Terrain USD has no default prim: {path}')
        if UsdGeom.GetStageUpAxis(terrain) != 'Z' or abs(UsdGeom.GetStageMetersPerUnit(terrain)-1.) > 1e-9:
            raise ValueError(f'Terrain USD must use metres and Z up: {path}')
        if any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in terrain.Traverse()):
            raise ValueError(f'Terrain USD must contain only static scenery: {path}')
        if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in terrain.Traverse()):
            raise ValueError(f'Terrain USD has no collision geometry: {path}')
    else:
        path = build_scene(scene_pcd(directory, scenario), directory / f'{scenario}.usdc',
                           **options)
    world.stage.DefinePrim('/World/layout','Xform').GetReferences().AddReference(str(path))
    print(f'[SUPER] Physical terrain scene: {path}', flush=True)


if __name__ == '__main__':
    import argparse
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=MAPS)
    args=parser.parse_args()
    directory=Path(__file__).resolve().parents[2]/'assets/super'
    print(build_scene(scene_pcd(directory, args.scenario), directory/f'{args.scenario}.usdc',
                      **scene_options(directory,args.scenario)))
