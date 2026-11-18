"""Geometry tests: local bag connectors must stay inside observed free space."""
from pathlib import Path
import sys
import unittest
import numpy as np
from scipy.spatial import cKDTree
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools/super'))
from dasan_route import connect_near_bag, sampled_segment


class ConnectorGeometry(unittest.TestCase):
    def test_solid_wall_cannot_be_crossed(self):
        def distance(p):
            p=np.asarray(p)
            return np.minimum.reduce([np.abs(p[:,0]-1.)-.1,p[:,2],3.5-p[:,2]])
        ref=np.array([[x,0.,1.7]for x in np.linspace(0.,2.,21)])
        audit={}
        self.assertIsNone(connect_near_bag(ref[0],ref[-1],ref,distance,.3,.5,.3,audit))
        self.assertEqual(audit['result'],'no_path')

    def test_unobserved_outdoor_space_cannot_be_used(self):
        ref=np.array([[0.,0.,1.7],[1.,0.,1.7]])
        audit={}
        self.assertIsNone(connect_near_bag(ref[0],ref[-1],ref,lambda p:np.full(len(p),100.),.3,.5,.3,audit))
        self.assertEqual(audit['result'],'missing_roof_or_floor')

    def test_observed_corner_can_be_connected(self):
        corners=[(-.8,-.8),(.8,-.8),(.8,2.2),(3.8,2.2),(3.8,3.8),(-.8,3.8),(-.8,-.8)]
        wall=[]
        for a,b in zip(corners[:-1],corners[1:]):
            for xy in np.linspace(a,b,max(2,int(np.linalg.norm(np.array(b)-a)/.05)+1)):
                wall.extend([[*xy,z]for z in np.arange(0.,3.51,.05)])
        for x in np.arange(-.8,3.81,.05):
            for y in np.arange(-.8,3.81,.05):
                if (x<=.8 or y>=2.2):wall.extend([[x,y,0.],[x,y,3.5]])
        tree=cKDTree(wall)
        def distance(p):return tree.query(p)[0]-.03
        ref=np.r_[sampled_segment(np.array([0.,1.,1.7]),np.array([0.,3.,1.7]),.1),sampled_segment(np.array([0.,3.,1.7]),np.array([2.,3.,1.7]),.1)]
        audit={};result=connect_near_bag(ref[0],ref[-1],ref,distance,.5,.75,.3,audit)
        self.assertIsNotNone(result,audit)
        dense=np.concatenate([sampled_segment(a,b)for a,b in zip(result[:-1],result[1:])])
        self.assertGreaterEqual(distance(dense).min(),.5-1e-8)
        self.assertLessEqual(cKDTree(ref[:,:2]).query(dense[:,:2])[0].max(),.86)
        np.testing.assert_allclose(result[[0,-1]],ref[[0,-1]])


if __name__=='__main__':unittest.main()
