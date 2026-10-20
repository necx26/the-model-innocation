import sys,math,unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws'))
from search_memory import SearchMemory,observable_floor_points,scan_clear_points


class SearchMemoryTests(unittest.TestCase):
    def test_empty_requires_three_distinct_frames(self):
        m=SearchMemory([(2,0),(3,0)])
        m.observe([0],[],1);m.observe([0],[],1)
        self.assertFalse(m.seen.any())
        m.observe([0],[],2);self.assertFalse(m.empty.any())
        m.observe([0],[],3)
        self.assertEqual(m.empty.tolist(),[True,False])
        self.assertEqual(m.summary()['unseen'],1)

    def test_seen_object_is_not_an_empty_cell(self):
        m=SearchMemory([(2,0),(3,0)])
        d={'x':2,'y':0,'color':'red'}
        for f in range(3):m.observe([0,1],[d],f)
        self.assertEqual(m.empty.tolist(),[False,True])
        self.assertTrue(m.present[0])

    def test_collection_is_local_and_survives_other_observations(self):
        m=SearchMemory([(2,0),(3,0)])
        m.collected((2,0));m.observe([1],[],1)
        self.assertEqual(m.cleared.tolist(),[True,False])
        self.assertFalse(m.seen[1])
        self.assertTrue(m.seen[0])

    def test_new_object_overrides_old_empty_record(self):
        m=SearchMemory([(2,0)])
        for f in range(3):m.observe([0],[],f)
        self.assertTrue(m.empty[0])
        m.observe([0],[{'x':2,'y':0,'color':'blue'}],4)
        self.assertFalse(m.empty[0]);self.assertTrue(m.present[0])

    def test_live_obstacle_keeps_hidden_floor_unknown(self):
        scan=np.full(361,np.inf);scan[179:182]=2.
        points=[(1.,0.),(4.,0.),(0.,4.)]
        valid=scan_clear_points(points,[0,0],0,scan,-math.pi,math.pi/180,.05,20.)
        self.assertEqual(valid.tolist(),[True,False,True])

    def test_occlusion_and_nonfloor_are_not_empty_evidence(self):
        points=np.array([(2.,0.),(.2,0.),(-2.,0.)])
        c,s=math.cos(.25),math.sin(.25)
        rot=np.array([[c,0,s],[0,1,0],[-s,0,c]])
        k=[600,0,480,0,600,270,0,0,1]
        floor=np.full((540,960,3),128,dtype=np.uint8)
        args=(points,[0,0,.4],rot,k,floor)
        self.assertEqual(observable_floor_points(*args,lambda a,b:True).tolist(),[0])
        self.assertEqual(observable_floor_points(*args,lambda a,b:False).tolist(),[])
        wall=np.zeros_like(floor);wall[:]=[20,100,180]
        self.assertEqual(observable_floor_points(points,[0,0,.4],rot,k,wall,lambda a,b:True).tolist(),[])


if __name__=='__main__':unittest.main()
