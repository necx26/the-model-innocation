"""Regressions for cars crossing the saved-state v2 partitions."""
import json,math,sys,tempfile,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ros_ws'))
from patrol_safety import patrol_reports,terrain_boxes,segment_box_clearance,boxes_clearance,swept_car_box
WORLD=ROOT/'ros_ws/src/yzbot/mybot_description/worlds/offic_room.world'
class PatrolSafetyTests(unittest.TestCase):
    def test_old_segment_hits_wall_between_clear_endpoints(self):
        p=dict(model='Move1',start=[-1.5,-10.4,.005],end=[-1.5,-8.,.005])
        self.assertFalse(patrol_reports([p],WORLD,False)[0]['passed'])
        with self.assertRaises(ValueError):patrol_reports([p],WORLD)
    def test_current_complete_sweeps_clear_terrain(self):
        ps=json.loads((ROOT/'ros_ws/scene_config.json').read_text())['patrols']
        reports=patrol_reports(ps,WORLD)
        self.assertTrue(all(r['passed'] for r in reports))
        self.assertGreater(reports[0]['minimum_car_wall_margin_m'],.6)
    def test_saved_state_pose_overrides_model_definition(self):
        xml='<sdf><world><model name="room"><pose>0 0 0 0 0 0</pose><link name="wall"><collision><geometry><box><size>1 1 2</size></box></geometry><pose>0 0 1 0 0 0</pose></collision></link></model><state><model name="room"><pose>5 0 0 0 0 0</pose></model></state></world></sdf>'
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'world';p.write_text(xml);box=terrain_boxes(p)[0]
            self.assertEqual(box['pose'][0],5.)
            self.assertEqual(segment_box_clearance((4,0),(6,0),box),0.)
            self.assertGreater(segment_box_clearance((-1,0),(1,0),box),3.)
    def test_car_width_can_hit_wall_with_clear_centreline(self):
        wall=dict(pose=(0.,.2,1.,0.),size=(.1,.05,2.))
        p=dict(start=[-1.,0.,0.],end=[1.,0.,0.])
        self.assertGreater(segment_box_clearance(p['start'],p['end'],wall),.17)
        self.assertEqual(boxes_clearance(swept_car_box(p),wall),0.)
    def test_rotated_box_clearance_is_rigid_transform_invariant(self):
        box=dict(pose=(0.,0.,1.,0.),size=(1.,1.,2.))
        expected=segment_box_clearance((2.,-1.),(2.,1.),box)
        c,s=math.cos(.7),math.sin(.7)
        def turn(p):return (3+c*p[0]-s*p[1],4+s*p[0]+c*p[1])
        rotated=dict(pose=(3.,4.,1.,.7),size=box['size'])
        self.assertAlmostEqual(segment_box_clearance(turn((2,-1)),turn((2,1)),rotated),expected)
if __name__=='__main__':unittest.main()
