from pathlib import Path
import os,sys,json,time
import numpy as np
from isaacsim import SimulationApp
build_only='--build-only' in sys.argv
app=SimulationApp({'headless':build_only,'width':1600,'height':1000,'sync_loads':False})
from pxr import Usd,UsdGeom,UsdShade,UsdPhysics,UsdLux,Sdf,Gf,Vt
import omni.usd,omni.ui as ui,carb
from omni.kit.viewport.utility import get_active_viewport
from scipy.spatial.transform import Rotation
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[3];stage_path=OUT/'frontier_textured.usdc'

def linear(rgb):return np.where(rgb<=.04045,rgb/12.92,((rgb+.055)/1.055)**2.4).astype(np.float32)
def texture_material(stage,index):
 path=f'/Frontier/Looks/Atlas_{index:03d}';mat=UsdShade.Material.Define(stage,path);surface=UsdShade.Shader.Define(stage,path+'/Surface');surface.CreateIdAttr('UsdPreviewSurface');surface.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0));surface.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.);surface.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
 reader=UsdShade.Shader.Define(stage,path+'/UV');reader.CreateIdAttr('UsdPrimvarReader_float2');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st');reader.CreateOutput('result',Sdf.ValueTypeNames.Float2)
 tex=UsdShade.Shader.Define(stage,path+'/Photo');tex.CreateIdAttr('UsdUVTexture');tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f'./textures/atlas_{index:03d}.jpg'));tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB');tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result');tex.CreateInput('wrapS',Sdf.ValueTypeNames.Token).Set('clamp');tex.CreateInput('wrapT',Sdf.ValueTypeNames.Token).Set('clamp');tex.CreateOutput('rgb',Sdf.ValueTypeNames.Float3)
 surface.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb');mat.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(),'surface')
 mdl=UsdShade.Shader.Define(stage,path+'/Photograph');mdl.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset);mdl.SetSourceAsset(Sdf.AssetPath('./photo_surface.mdl'),'mdl');mdl.SetSourceAssetSubIdentifier('photo_surface','mdl');mdl.CreateInput('photo',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f'./textures/atlas_{index:03d}.jpg'));mdl.CreateInput('gain',Sdf.ValueTypeNames.Float).Set(1.);mdl.CreateOutput('out',Sdf.ValueTypeNames.Token);mat.CreateSurfaceOutput('mdl').ConnectToSource(mdl.ConnectableAPI(),'out');return mat

def vertex_material(stage):
 path='/Frontier/Looks/MeasuredPointColor';mat=UsdShade.Material.Define(stage,path);surface=UsdShade.Shader.Define(stage,path+'/Surface');surface.CreateIdAttr('UsdPreviewSurface');surface.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0));surface.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
 reader=UsdShade.Shader.Define(stage,path+'/Color');reader.CreateIdAttr('UsdPrimvarReader_float3');reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('displayColor');reader.CreateInput('fallback',Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(.35,.35,.35));reader.CreateOutput('result',Sdf.ValueTypeNames.Float3);surface.CreateInput('emissiveColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(reader.ConnectableAPI(),'result');mat.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(),'surface');return mat

def camera_at(stage,name,eye,forward,K):
 camera=UsdGeom.Camera.Define(stage,f'/Frontier/Cameras/{name}');eye=np.asarray(eye);forward=np.asarray(forward);forward/=np.linalg.norm(forward);right=np.cross(forward,[0,0,1.]);right/=np.linalg.norm(right);up=np.cross(right,forward);matrix=np.column_stack([right,up,-forward]);q=Rotation.from_matrix(matrix).as_quat();xf=UsdGeom.Xformable(camera);xf.AddTranslateOp().Set(Gf.Vec3d(*eye));xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]),Gf.Vec3d(*q[:3])));focal=24.;camera.CreateFocalLengthAttr(focal);camera.CreateHorizontalApertureAttr(float(focal*640/K[0,0]));camera.CreateVerticalApertureAttr(float(focal*512/K[1,1]));camera.CreateClippingRangeAttr(Gf.Vec2f(.05,300.));return str(camera.GetPath())

def export():
 m=np.load(OUT/'mesh.npz');tex=np.load(OUT/'textured.npz');v=m['vertices'];tri=m['triangles'];normal=m['normals'];rgb=linear(m['colors']);material=tex['material'];uv=tex['uv'];bands=np.floor(v[tri].mean(axis=1)[:,2]/2.).astype(int)
 stage=Usd.Stage.CreateNew(str(stage_path));UsdGeom.SetStageMetersPerUnit(stage,1.);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);root=UsdGeom.Xform.Define(stage,'/Frontier').GetPrim();stage.SetDefaultPrim(root);root.SetCustomDataByKey('purpose','Map-only photographic visualization; no flight experiment');root.SetCustomDataByKey('source','FAST-LIVO2 measured Frontier bag and real camera images')
 fallback=texture_material(stage,75)
 for path,input_name in [('/Frontier/Looks/Atlas_075/Photo','file'),('/Frontier/Looks/Atlas_075/Photograph','photo')]:
  UsdShade.Shader(stage.GetPrimAtPath(path)).GetInput(input_name).Set(Sdf.AssetPath('./textures/measured_colors.png'))
 fallback_ids=np.flatnonzero(material<0);number=np.arange(len(fallback_ids));uv[fallback_ids,:,0]=((number%1024+.5)/1024)[:,None];uv[fallback_ids,:,1]=(1-(number//1024+.5)/1024)[:,None]
 mats={-1:fallback}
 for i in range(int(material.max())+1):mats[i]=texture_material(stage,i)
 chunks=[]
 for band in sorted(np.unique(bands)):
  group=UsdGeom.Xform.Define(stage,f'/Frontier/Terrain/Height_{band+10:02d}');group.GetPrim().SetCustomDataByKey('height_min_m',float(band*2.))
  for mat in sorted(np.unique(material[bands==band])):
   ids=np.flatnonzero((bands==band)&(material==mat));indices=tri[ids].reshape(-1);positions=v[indices];name='MeasuredColor'if mat<0 else f'Photo_{mat:03d}'
   mesh=UsdGeom.Mesh.Define(stage,str(group.GetPath())+'/'+name);mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(positions));mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(ids),3,np.int32)));mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(np.arange(len(indices),dtype=np.int32)));mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True);mesh.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normal[indices]));mesh.SetNormalsInterpolation('vertex');mesh.CreateExtentAttr(UsdGeom.PointBased(mesh).ComputeExtent(positions))
   UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.faceVarying).Set(Vt.Vec2fArray.FromNumpy(uv[ids].reshape(-1,2)))
   UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mats[int(mat)]);UsdPhysics.CollisionAPI.Apply(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none');chunks.append(dict(path=str(mesh.GetPath()),triangles=len(ids),band=int(band),atlas=int(mat)))
  print('EXPORTED_HEIGHT',int(band),flush=True)
 cams=np.load(OUT/'cameras.npz');A=cams['rotation'];b=cams['translation'];origins=-np.einsum('nji,nj->ni',A,b);directions=A[:,2,:];K=cams['K'];profiles={}
 for name,target,heading in [('Corridor_Lower',[-9.2,6.,5.3],[0,1,0]),('Corridor_Upper',[-9.2,7.,13.3],[0,1,0]),('Stair_Landing',[-.7,3.,11.3],[0,1,0])]:
  direction_score=directions@heading;score=np.linalg.norm(origins-np.array(target),axis=1)+np.maximum(.8-direction_score,0)*15.;index=int(score.argmin());path=camera_at(stage,name,origins[index],directions[index],K);profiles[name]=dict(path=path,source_image=int(index),position=origins[index].tolist(),forward=directions[index].tolist())
 camera_at(stage,'Overview',[-38.,-22.,34.],np.array([-5.,18.,11.])-np.array([-38.,-22.,34.]),K)
 light=UsdLux.DomeLight.Define(stage,'/Frontier/Light');light.CreateIntensityAttr(150.)
 stage.GetRootLayer().Save();(OUT/'viewer.json').write_text(json.dumps(dict(stage=str(stage_path),profiles=profiles,chunks=chunks),indent=2));print('USD_SAVED',stage_path,flush=True)

if not stage_path.exists():export()
if build_only:
 app.close();raise SystemExit(0)
context=omni.usd.get_context();context.open_stage(str(stage_path))
for _ in range(40):app.update()
viewport=get_active_viewport();viewport.camera_path='/Frontier/Cameras/Corridor_Lower';profiles=json.loads((OUT/'viewer.json').read_text())['profiles'];window=ui.Window('Frontier terrain',width=300,height=300,position_x=1100,position_y=420)
label=None

def switch(name):
 viewport.camera_path='/Frontier/Cameras/'+name
 print('CAMERA_SWITCH',name,UsdGeom.Xformable(context.get_stage().GetPrimAtPath(viewport.camera_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation(),flush=True)
 if label:label.text=name.replace('_',' ')

def cutaway(enabled):
 stage=context.get_stage()
 for prim in stage.GetPrimAtPath('/Frontier/Terrain').GetChildren():
  z=prim.GetCustomDataByKey('height_min_m');imageable=UsdGeom.Imageable(prim)
  if enabled and z>=6.:imageable.MakeInvisible()
  else:imageable.MakeVisible()

with window.frame:
 with ui.VStack(spacing=7):
  ui.Label('Frontier — photographic surface',height=25)
  label=ui.Label('Lower corridor',height=20)
  ui.Button('Lower corridor',clicked_fn=lambda:switch('Corridor_Lower'),height=28)
  ui.Button('Upper corridor',clicked_fn=lambda:switch('Corridor_Upper'),height=28)
  ui.Button('Stair landing',clicked_fn=lambda:switch('Stair_Landing'),height=28)
  ui.Button('Whole building',clicked_fn=lambda:switch('Overview'),height=28)
  ui.Button('Cutaway above lower floor',clicked_fn=lambda:cutaway(True),height=28)
  ui.Button('Show all floors',clicked_fn=lambda:cutaway(False),height=28)
window.position_x=1100;window.position_y=420
stage=context.get_stage()
meshes=[UsdGeom.Mesh(p)for p in stage.Traverse()if p.IsA(UsdGeom.Mesh)]
triangles=sum(len(m.GetFaceVertexCountsAttr().Get())for m in meshes)
rigid=[str(p.GetPath())for p in stage.Traverse()if p.HasAPI(UsdPhysics.RigidBodyAPI)]
assert triangles==1486492 and not rigid
assert not any('quadrotor'in str(p.GetPath()).lower()for p in stage.Traverse())
inspection=dict(stage=str(stage_path),mesh_chunks=len(meshes),triangles=triangles,photographic_atlases=75,rigid_bodies=rigid,drone_present=False,timeline_playing=False,mode='static map-only viewer; no flight stack started')
(OUT/'inspection.json').write_text(json.dumps(inspection,indent=2))
print('INSPECTION',json.dumps(inspection),flush=True)
print('VIEWER_READY: MAP ONLY; no vehicle, ROS nodes, PX4 or mission',flush=True)
requests=ROOT/'log/frontier-20261119/visual-terrain/view-command.json';previous=''
try:
 while app.is_running():
  app.update()
  if requests.exists():
   text=requests.read_text()
   if text!=previous:
    previous=text
    try:
     cmd=json.loads(text)
     if 'camera'in cmd:switch(cmd['camera'])
     if 'cutaway'in cmd:cutaway(cmd['cutaway'])
    except Exception as e:print('VIEW_COMMAND_ERROR',repr(e),flush=True)
except KeyboardInterrupt:pass
finally:app.close()
