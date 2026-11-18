import sys,unittest,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/super'))
from bag_map_alignment import BagMapAlignment,rotation,cloud_xyz


class Registration(unittest.TestCase):
    def test_recovers_lio_drift_without_truth_or_changing_controller_frame(self):
        rng=np.random.default_rng(8);pieces=[]
        for axis,extent in [(0,3.),(1,5.),(2,3.)]:
            for sign in [-1,1]:
                p=rng.uniform([-3,-5,0],[3,5,3],(5000,3))
                p[:,axis]=sign*extent if axis<2 else (0. if sign<0 else extent)
                pieces.append(p)
        reference=np.concatenate(pieces);yaw=math.radians(2.);translation=np.array([.2,-.9,.1])
        source=(reference-translation)@rotation(yaw)+rng.normal(0,.008,reference.shape)
        source=np.r_[source,rng.uniform([-3,-5,0],[3,5,3],(500,3))]
        alignment=BagMapAlignment(reference);result=alignment.register(source,[0.,0.,1.5])
        self.assertTrue(result['accepted'],result)
        self.assertLess(abs(alignment.yaw-yaw),math.radians(.5))
        self.assertLess(np.linalg.norm(alignment.translation-translation),.07)
        targets=[[0.,0.,1.5],[1.,2.,1.5]];converted,yaws=alignment.route_in_live_frame(targets,[0.,math.pi/2],[0.,0.,1.5])
        self.assertEqual(converted[0],[0.,0.,1.5])
        recovered=alignment.rotation@converted[1]+alignment.translation
        np.testing.assert_allclose(recovered,targets[1],atol=1e-8)
        self.assertAlmostEqual(yaws[1]+alignment.yaw,math.pi/2,places=5)

    def test_recovers_tilt_that_changes_height_along_a_long_corridor(self):
        from scipy.spatial.transform import Rotation
        rng=np.random.default_rng(5);pieces=[]
        for axis,extent in [(0,3.),(1,5.),(2,1.5)]:
            for sign in [-1,1]:
                p=rng.uniform([-3,-5,-1.5],[3,5,1.5],(4000,3));p[:,axis]=sign*extent
                pieces.append(p+[0.,35.,9.])
        reference=np.concatenate(pieces)
        true_rotation=Rotation.from_euler('xyz',[1.7,-1.2,2.],degrees=True).as_matrix()
        translation=np.array([.2,-.4,.1]);source=(reference-translation)@true_rotation
        a=BagMapAlignment(reference);r=a.register(source,[0.,35.,9.])
        self.assertTrue(r['accepted'],r)
        self.assertLess(Rotation.from_matrix(a.rotation@true_rotation.T).magnitude(),math.radians(.3))
        self.assertLess(np.linalg.norm(a.translation-translation),.07)

    def test_ambiguous_plane_does_not_update_alignment(self):
        rng=np.random.default_rng(2);reference=rng.uniform(-3,3,(4000,3));reference[:,2]=0.
        a=BagMapAlignment(reference);r=a.register(reference+[0.,0.,.3],[0.,0.,1.5])
        self.assertFalse(r['accepted']);np.testing.assert_array_equal(a.translation,[0.,0.,0.])

    def test_recovery_keeps_far_end_and_all_unvisited_stairs(self):
        a=BagMapAlignment(np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]))
        points=[[0.,0.,1.],[0.,10.,1.],[0.,40.,1.],[0.,0.,1.],[0.,0.,5.]]
        suffix,yaws=a.remaining_route(points,[0.,0.,math.pi,math.pi,0.],1,[0.,12.,1.])
        self.assertEqual(suffix[1:],points[2:]);self.assertEqual(suffix[0],[0.,12.,1.])
        self.assertEqual(yaws[1:],[math.pi,math.pi,0.])
        with self.assertRaises(ValueError):a.remaining_route(points,[0.]*5,4,[0.,0.,5.])

    def test_organized_big_endian_cloud_preserves_row_padding(self):
        from types import SimpleNamespace as S
        data=bytearray(40)
        for i,p in enumerate([[1.,2.,3.],[4.,5.,6.]]):
            np.ndarray((3,),dtype='>f4',buffer=data,offset=i*20)[:]=p
        msg=S(fields=[S(name=n,offset=i*4,datatype=7)for i,n in enumerate('xyz')],
              is_bigendian=True,height=2,width=1,row_step=20,point_step=16,data=data)
        np.testing.assert_array_equal(cloud_xyz(msg),[[1.,2.,3.],[4.,5.,6.]])

    def test_delayed_progress_does_not_send_passed_straight_vertices_again(self):
        a=BagMapAlignment(np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]))
        points=[[0.,0.,1.],[0.,10.,1.],[0.,20.,1.],[0.,40.,1.],[0.,0.,1.]]
        suffix,_=a.remaining_route(points,[0.]*5,0,[0.,25.,1.])
        self.assertEqual(suffix,[[0.,25.,1.],[0.,40.,1.],[0.,0.,1.]])

    def test_delayed_progress_cannot_bypass_far_end_or_unvisited_corner(self):
        a=BagMapAlignment(np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]))
        points=[[0.,0.,1.],[0.,40.,1.],[0.,0.,1.]]
        suffix,_=a.remaining_route(points,[0.]*3,0,[0.,42.,1.])
        self.assertEqual(suffix[1:],points[1:])
        corner=[[0.,0.,1.],[0.,5.,1.],[5.,5.,3.]]
        suffix,_=a.remaining_route(corner,[0.]*3,0,[1.,6.,1.])
        self.assertEqual(suffix[1:],corner[1:])


if __name__=='__main__':unittest.main()
