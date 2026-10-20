import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws'))
from cube_perception import fit_floor_cube_box


class CubePoseFitTests(unittest.TestCase):
    def camera(self,yaw):
        c,s=math.cos(yaw),math.sin(yaw);p=.57
        rotation=np.array([[c,-s,0],[s,c,0],[0,0,1]])@np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
        return np.array([.2,-.1,.48]),rotation,[338.,0,480.,0,338.,270.,0,0,1]
    def test_close_cube_centre_is_recovered_from_diagonal_views(self):
        for yaw in (0.,.6,2.4068,-2.4):
            origin,rotation,k=self.camera(yaw)
            true=origin[:2]+.85*np.array([math.cos(yaw),math.sin(yaw)])
            vertices=np.array([(true[0]+dx,true[1]+dy,z) for dx in (-.025,.025) for dy in (-.025,.025) for z in (0.,.05)])
            camera=(vertices-origin)@rotation
            u=k[2]-k[0]*camera[:,1]/camera[:,0];v=k[5]-k[4]*camera[:,2]/camera[:,0]
            left,top=math.floor(min(u)),math.floor(min(v));right,bottom=math.ceil(max(u)),math.ceil(max(v))
            result=fit_floor_cube_box([left,top,right-left+1,bottom-top+1],k,origin,rotation,true+(.025,.020))
            self.assertIsNotNone(result)
            self.assertLess(np.linalg.norm(np.array(result[:2])-true),.004)
    def test_non_cube_shape_and_unresolved_far_speck_do_not_refine_pose(self):
        origin,rotation,k=self.camera(0.)
        self.assertIsNone(fit_floor_cube_box([470,250,80,12],k,origin,rotation,(1.,0.)))
        self.assertIsNone(fit_floor_cube_box([480,270,4,5],k,origin,rotation,(1.,0.)))


if __name__=='__main__':unittest.main()
