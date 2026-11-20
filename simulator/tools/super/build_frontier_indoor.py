"""Build the static Frontier infill from measured storeys and indoor floor/ceiling returns."""
import argparse
from pathlib import Path

import numpy as np
from scipy import ndimage

from pcd_scene import read_pcd


# Local u/v axes follow the two long corridor walls in the aligned Frontier PCD.
YAW_DEG = 1.58
STEP = .20
U = np.arange(-21., 11. + STEP / 2, STEP)
V = np.arange(-13., 51. + STEP / 2, STEP)
# Measured slab modes, with the repair recessed behind intact photographic surfaces.
FLOORS = np.array([-.13, 3.92, 7.97, 12.02, 16.02, 20.02])
CEILINGS = np.array([2.52, 6.52, 10.57, 14.62, 18.62, 22.62])
FLOOR_BACKING = FLOORS - .15
CEILING_BACKING = CEILINGS + .22
ROOF = 25.25
THICKNESS = .12
TILE_FACES = np.array([[0,3,2,1], [4,5,6,7], [0,1,5,4],
                       [1,2,6,5], [2,3,7,6], [3,0,4,7]], dtype=np.int32)


def local_xy(points):
    angle = np.deg2rad(YAW_DEG)
    return points[:, :2] @ np.array([[np.cos(angle), -np.sin(angle)],
                                     [np.sin(angle), np.cos(angle)]])


def indoor_volume(points):
    uv = local_xy(points)
    u, v = np.meshgrid((U[1:] + U[:-1]) / 2, (V[1:] + V[:-1]) / 2, indexing='ij')
    z = np.unique(np.r_[FLOOR_BACKING, CEILING_BACKING, ROOF])
    zm = (z[:-1] + z[1:]) / 2
    volume = np.zeros((*u.shape, len(zm)), dtype=bool)

    def rectangle(left, right, south, north):
        return (left < u) & (u < right) & (south < v) & (v < north)

    def returns(height):
        at_height = uv[np.abs(points[:, 2] - height) < .20]
        return np.histogram2d(*at_height.T, bins=(U, V))[0] > 0

    # The stairwell spans the inter-storey voids; floor plates terminate at its opening.
    shaft = rectangle(-2.25, 2.03, 1.30, 7.92)
    volume[shaft, :] = True
    disk = ndimage.iterate_structure(ndimage.generate_binary_structure(2, 1), 2)
    for level, (floor, ceiling) in enumerate(zip(FLOORS, CEILINGS)):
        if level < 5:
            west = -9.56 if level == 0 else -10.78
            core = (rectangle(west, -6.58, -10.86, 10.86)
                    | rectangle(-9.60, -6.62, 10.26, 47.34)
                    | rectangle(west, 2.08, -2.14, 1.79)
                    | rectangle(-7.2, -1.82, 4.53, 6.50))
        else:
            core = rectangle(-6.48, 2.08, -2.13, 1.79)
        # Extend into measured branches only where both horizontal surfaces are supported.
        paired = ndimage.binary_closing(returns(floor), disk) & ndimage.binary_closing(returns(ceiling), disk)
        paired = ndimage.binary_opening(ndimage.binary_closing(paired, disk), disk)
        supported = ndimage.binary_dilation(paired, iterations=1)
        outline = ndimage.binary_propagation(core | shaft, mask=core | shaft | supported)
        volume |= outline[:, :, None] & ((zm > FLOOR_BACKING[level]) & (zm < CEILING_BACKING[level]))
    return (U, V, z), volume


def rectangles(mask):
    mask = mask.copy()
    for r in range(mask.shape[0]):
        for c in np.flatnonzero(mask[r]):
            if not mask[r, c]:
                continue
            end = c + 1
            while end < mask.shape[1] and mask[r, end]:
                end += 1
            bottom = r + 1
            while bottom < mask.shape[0] and mask[bottom, c:end].all():
                bottom += 1
            mask[r:bottom, c:end] = False
            yield r, bottom, c, end


def boundary_panels(edges, volume):
    panels = {'Floor': [], 'Ceiling': [], 'Wall': []}
    for axis in range(3):
        other = [i for i in range(3) if i != axis]
        layers = np.moveaxis(volume.astype(np.int8), axis, 0)
        transitions = np.diff(np.pad(layers, ((1,1),(0,0),(0,0))), axis=0)
        for i, plane in enumerate(transitions):
            for sign in (-1, 1):
                kind = ('Floor' if sign == 1 else 'Ceiling') if axis == 2 else 'Wall'
                for a,b,c,d in rectangles(plane == sign):
                    low, high = np.zeros(3), np.zeros(3)
                    # Extrude away from the free volume. A 4 mm edge overlap seals panel seams.
                    plane_at = edges[axis][i]
                    low[axis] = min(plane_at, plane_at - sign * THICKNESS)
                    high[axis] = max(plane_at, plane_at - sign * THICKNESS)
                    low[other] = [edges[other[0]][a]-.004, edges[other[1]][c]-.004]
                    high[other] = [edges[other[0]][b]+.004, edges[other[1]][d]+.004]
                    panels[kind].append((low,high))
    return panels


def write_infill(output, panels, points, colors):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
    temp = output.with_suffix('.tmp.usdc')
    stage = Usd.Stage.CreateNew(str(temp))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    root = UsdGeom.Xform.Define(stage, '/IndoorInfill')
    root.AddRotateZOp().Set(YAW_DEG)
    stage.SetDefaultPrim(root.GetPrim())
    uv = local_xy(points)
    hallway = (uv[:,0] > -9.2) & (uv[:,0] < -7.2) & (uv[:,1] > 12) & (uv[:,1] < 40)
    for kind, boxes in panels.items():
        if kind == 'Wall':
            sample = (abs(uv[:,0]+9.25)<.18) & (uv[:,1]>12) & (uv[:,1]<40) & (abs(points[:,2]-9.3)<.85)
        else:
            heights = FLOORS if kind == 'Floor' else CEILINGS
            sample = hallway & (np.abs(points[:,2,None] - heights).min(axis=1)<.15)
        srgb = np.median(colors['rgb'][sample & colors['direct']],axis=0)/255.
        color = np.where(srgb<=.04045, srgb/12.92, ((srgb+.055)/1.055)**2.4)
        material = UsdShade.Material.Define(stage, '/IndoorInfill/Materials/'+kind)
        shader = UsdShade.Shader.Define(stage, str(material.GetPath())+'/Surface')
        shader.CreateIdAttr('UsdPreviewSurface')
        shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.92)
        shader.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*(color*.18)))
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
        xyz = []
        for (x,y,z),(X,Y,Z) in boxes:
            xyz.extend(((x,y,z),(X,y,z),(X,Y,z),(x,Y,z),(x,y,Z),(X,y,Z),(X,Y,Z),(x,Y,Z)))
        xyz = np.asarray(xyz,dtype=np.float32)
        mesh = UsdGeom.Mesh.Define(stage, '/IndoorInfill/'+kind)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(xyz))
        idx = TILE_FACES[None,:,:] + 8*np.arange(len(boxes),dtype=np.int32)[:,None,None]
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(idx.ravel()))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(idx.size//4,4,np.int32)))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateExtentAttr([Gf.Vec3f(*xyz.min(axis=0).tolist()), Gf.Vec3f(*xyz.max(axis=0).tolist())])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
    stage.GetRootLayer().Save()
    temp.replace(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets',type=Path,default=Path(__file__).resolve().parents[2]/'assets/super')
    parser.add_argument('--audit-volume',type=Path,help='Save the designed interior cells for independent flight analysis')
    args = parser.parse_args()
    p = read_pcd(args.assets/'frontier.pcd')
    edges, volume = indoor_volume(p)
    if ndimage.label(volume)[1] != 1:
        raise ValueError('The indoor stairwell and storeys must form one connected space')
    panels = boundary_panels(edges,volume)
    out = args.assets/'frontier_visual/frontier_indoor_infill.usdc'
    with np.load(args.assets/'frontier_color.npz') as colors:
        write_infill(out,panels,p,colors)
    if args.audit_volume:
        np.savez_compressed(args.audit_volume, u=edges[0], v=edges[1], z=edges[2], inside=volume, yaw_deg=YAW_DEG)
    print(out, {name:len(items) for name,items in panels.items()})


if __name__ == '__main__':
    main()
