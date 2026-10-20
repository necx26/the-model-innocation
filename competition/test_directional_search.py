import math
import sys
import unittest
from pathlib import Path
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws'))
from mission_strategy import MissionMap,wrap


class DirectionalSearchTests(unittest.TestCase):
    def grid(self,points,views):
        grid=MissionMap.__new__(MissionMap)
        grid.search_points=np.asarray(points,dtype=float)
        grid.spawnable=np.ones(len(points),dtype=bool)
        grid.views=views
        grid.distances=lambda point:point
        grid.distance=lambda start,end:math.dist(start[:2],end[:2])
        return grid

    def test_symmetric_unknown_rooms_do_not_produce_empty_arrival_view(self):
        points=[(1.,2.),(1.,-2.)]
        grid=self.grid(points,[dict(point=(1.,0.),visible=np.array([0,1]))])
        result=grid.next_view((0.,0.,0.),np.zeros(2,dtype=bool),{},directional=True)
        _,_,pose,gain,_=result
        visible=sum(abs(wrap(math.atan2(y-pose[1],x-pose[0])-pose[2]))<.58 for x,y in points)
        self.assertEqual(gain,1)
        self.assertEqual(visible,gain)

    def test_equal_gain_prefers_forward_trip_over_turnaround(self):
        grid=self.grid([(-3.,0.),(3.,0.)],
                       [dict(point=(-1.,0.),visible=np.array([0])),
                        dict(point=(1.,0.),visible=np.array([1]))])
        result=grid.next_view((0.,0.,0.),np.zeros(2,dtype=bool),{},directional=True)
        self.assertEqual(result[1],1)
        self.assertLess(abs(result[2][2]),.01)


if __name__=='__main__':unittest.main()
