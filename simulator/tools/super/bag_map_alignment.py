"""Register live LIO points to the recorded map for a bag route source only.

The controller and SUPER stay in the unmodified live LIO frame. This module
never reads simulation truth and never injects reference points into ROG-Map.
Live exploration publishers already share the online map frame and do not use it.
"""
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation


def cloud_xyz(message):
    fields={f.name:f for f in message.fields}
    if any(k not in fields or fields[k].datatype!=7 for k in ('x','y','z')):
        raise ValueError('Expected float32 XYZ cloud')
    endian='>' if message.is_bigendian else '<'
    points=np.column_stack([np.ndarray((message.height,message.width),
        dtype=endian+'f4',buffer=message.data,offset=fields[k].offset,
        strides=(message.row_step,message.point_step)).ravel() for k in ('x','y','z')])
    return points[np.isfinite(points).all(axis=1)].astype(float)


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])


class BagMapAlignment:
    def __init__(self,reference,resolution=.15):
        reference=np.asarray(reference,dtype=float)
        reference=reference[np.isfinite(reference).all(axis=1)]
        _,indices=np.unique(np.floor(reference/resolution).astype(np.int64),axis=0,return_index=True)
        self.reference=reference[indices];self.tree=cKDTree(self.reference)
        self.rotation=np.eye(3);self.translation=np.zeros(3);self.normal_cache={}
        self.resolution=resolution

    @property
    def yaw(self):return math.atan2(self.rotation[1,0],self.rotation[0,0])

    def normals(self,indices):
        unique,inverse=np.unique(indices,return_inverse=True)
        missing=[int(i) for i in unique if int(i) not in self.normal_cache]
        if missing:
            _,neighbors=self.tree.query(self.reference[missing],k=16)
            patches=self.reference[neighbors];centered=patches-patches.mean(axis=1,keepdims=True)
            covariance=np.einsum('nki,nkj->nij',centered,centered)
            values,vectors=np.linalg.eigh(covariance)
            for i,v,n in zip(missing,values,vectors[:,:,0]):
                self.normal_cache[i]=(n,bool(v[0]<.25*max(v[1],1e-12)))
        normal=np.array([self.normal_cache[int(i)][0] for i in unique])[inverse]
        planar=np.array([self.normal_cache[int(i)][1] for i in unique])[inverse]
        return normal,planar

    def register(self,cloud,body_position):
        source=np.asarray(cloud,dtype=float);body=np.asarray(body_position,dtype=float)
        ranges=np.linalg.norm(source-body,axis=1)
        source=source[np.isfinite(source).all(axis=1)&(ranges>.8)&(ranges<12.)]
        _,indices=np.unique(np.floor(source/.2).astype(np.int64),axis=0,return_index=True)
        source=source[indices]
        if len(source)>5000:source=source[np.linspace(0,len(source)-1,5000,dtype=int)]
        if len(source)<200:return dict(accepted=False,reason='insufficient_live_points',points=len(source))
        Rmap=self.rotation.copy();translation=self.translation.copy();initial_body=Rmap@body+translation
        condition=0.
        for _ in range(20):
            points=source@Rmap.T+translation;center=Rmap@body+translation
            distances,ids=self.tree.query(points);normal,planar=self.normals(ids)
            residual=np.sum(normal*(points-self.reference[ids]),axis=1)
            keep=(distances<1.2)&planar&(abs(residual)<.6)
            if keep.sum()<150:return dict(accepted=False,reason='insufficient_plane_matches')
            p=points[keep]-center;n=normal[keep];r=residual[keep]
            # Scale rotation unknowns as displacement at a six-metre lever arm.
            # Roll/pitch matter: a small map-frame tilt grows into metres of
            # height error at the far end of a long corridor.
            A=np.column_stack([np.cross(p,n)/6.,n])
            weights=np.minimum(1.,.1/np.maximum(abs(r),1e-12))
            H=A.T@(weights[:,None]*A);eigen=np.linalg.eigvalsh(H)
            condition=float(eigen[0]/max(eigen[-1],1e-12))
            if condition<1e-4:return dict(accepted=False,reason='unobservable_registration',condition=condition)
            step=np.linalg.solve(H+np.eye(6)*1e-6,-A.T@(weights*r))
            angles=step[:3]/6.;angles*=min(1.,math.radians(.5)/max(np.linalg.norm(angles),1e-12))
            delta=step[3:];delta*=min(1.,.2/max(np.linalg.norm(delta),1e-12))
            R=Rotation.from_rotvec(angles).as_matrix()
            translation=R@(translation-center)+center+delta;Rmap=R@Rmap
            if np.linalg.norm(delta)<.001 and np.linalg.norm(angles)<1e-4:break
        points=source@Rmap.T+translation;distances,ids=self.tree.query(points)
        inliers=distances<.3;fraction=float(inliers.mean())
        rmse=float(np.sqrt(np.mean(distances[inliers]**2)))if inliers.any()else math.inf
        body_delta=Rmap@body+translation-initial_body
        angle_delta=float(Rotation.from_matrix(Rmap@self.rotation.T).magnitude())
        accepted=(fraction>=.55 and rmse<=.18 and np.linalg.norm(body_delta)<=2. and
                  angle_delta<=math.radians(5.) and np.isfinite(translation).all())
        result=dict(accepted=bool(accepted),reason='accepted'if accepted else 'registration_quality',
                    points=len(source),inlier_fraction=fraction,rmse_m=rmse,condition=condition,
                    yaw_rad=math.atan2(Rmap[1,0],Rmap[0,0]),
                    rotation_xyzw=Rotation.from_matrix(Rmap).as_quat().tolist(),
                    rotation_correction_rad=angle_delta,
                    translation_m=translation.tolist(),body_correction_m=body_delta.tolist())
        if accepted:self.rotation=Rmap;self.translation=translation
        return result

    def route_in_live_frame(self,points,yaws,body_position):
        points=(np.asarray(points,dtype=float)-self.translation)@self.rotation
        points[0]=body_position
        heading=[None if y is None else np.array([math.cos(y),math.sin(y),0.])@self.rotation for y in yaws]
        return points.tolist(),[None if v is None else math.atan2(v[1],v[0])for v in heading]

    def remaining_route(self,points,yaws,segment,body_position):
        """Keep every unvisited vertex, including the far-end reversal."""
        if segment<0 or segment>=len(points)-1:raise ValueError('Invalid active path segment')
        current=self.rotation@np.asarray(body_position)+self.translation
        # Progress feedback can lag behind a moving vehicle during registration.
        # Drop only vertices physically passed on the same ordered segment;
        # never jump an unvisited corner or the far-end reversal.
        while segment<len(points)-2:
            a,b,c=np.asarray(points[segment]),np.asarray(points[segment+1]),np.asarray(points[segment+2])
            incoming=b-a;length=np.linalg.norm(incoming)
            if length<1e-6:break
            direction=incoming/length;along=float((current-a)@direction)
            lateral=np.linalg.norm(current-(a+along*direction))
            outgoing=c-b
            reversal=(incoming@outgoing)<-.5*length*np.linalg.norm(outgoing)
            if along<length or lateral>.35 or (reversal and np.linalg.norm(current-b)>.35):break
            segment+=1
        return [current.tolist(),*points[segment+1:]],[yaws[segment],*yaws[segment+1:]]
