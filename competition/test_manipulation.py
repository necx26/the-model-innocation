import math,sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros_ws'))
from manipulation import capture_geometry,forward_capture_stops,gripper_feedback_ready,floor_chassis_clearance

class ManipulationTests(unittest.TestCase):
    def test_front_sides_and_rear_are_proposals_not_under_chassis(self):
        self.assertTrue(capture_geometry((0,0,0),(.24,0)))
        self.assertTrue(capture_geometry((0,0,0),(.26,0)))
        self.assertFalse(capture_geometry((0,0,0),(.26,0),max_radial=.385))
        self.assertTrue(capture_geometry((0,0,0),(0,.31)))
        self.assertTrue(capture_geometry((0,0,0),(0,-.31)))
        self.assertTrue(capture_geometry((0,0,0),(-.42,0)))
        self.assertFalse(capture_geometry((0,0,0),(0,.2)))
        self.assertFalse(capture_geometry((0,0,0),(.5,0)))
    def test_floor_cube_in_front_of_real_body_is_not_hidden_by_stowed_arm_envelope(self):
        self.assertGreater(floor_chassis_clearance((.15,0.)),.04)
        self.assertTrue(capture_geometry((0,0,0),(.15,0.)))
        for point in ((0.,0.),(-.2,.1),(0.,.185),(0.,-.185)):
            self.assertLess(floor_chassis_clearance(point),.04)
            self.assertFalse(capture_geometry((0,0,0),point))
    def test_reach_geometry_rotates_with_base(self):
        for yaw in (0.,.7,2.9):
            target=(-.31*math.sin(yaw),.31*math.cos(yaw))
            self.assertTrue(capture_geometry((0.,0.,yaw),target))
    def test_forward_motion_can_stop_for_lateral_cube_without_turning(self):
        values=forward_capture_stops((0,0,0),(.7,.3))
        self.assertTrue(values)
        distance,pose=values[0]
        self.assertGreater(distance,0.)
        self.assertEqual(pose[2],0.)
        self.assertTrue(capture_geometry(pose,(.7,.3)))
        self.assertEqual(forward_capture_stops((0,0,0),(3.,0.)),[])
    def test_jaw_needs_fresh_feedback_and_bounded_contact_progress(self):
        self.assertFalse(gripper_feedback_ready(.015,.015,.015,.3,False))
        self.assertFalse(gripper_feedback_ready(.015,.015,.015,.1,True))
        self.assertTrue(gripper_feedback_ready(.015,.014,.015,.23,True))
        self.assertTrue(gripper_feedback_ready(.008,.008,.015,.35,True))
        self.assertFalse(gripper_feedback_ready(.008,.006,.015,.35,True))
        self.assertFalse(gripper_feedback_ready(.008,.008,0.,.35,True))

if __name__=='__main__':unittest.main()
