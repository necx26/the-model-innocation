import unittest,sys,math
from pathlib import Path
import xml.etree.ElementTree as ET
import cv2,numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'ros_ws'))
from robot_geometry import NAV_FOOTPRINT, point_in_footprint, lidar_front_lane_clear, reachable_drop_candidates, departure_direction, delivery_region_approaches, footprint_clearance, usable_drop_candidates, square_gripper_yaw, downward_gripper_quaternion
from robot_geometry import front_corridor_distance,braking_speed
from robot_geometry import grasp_preflight_valid


class GeometryTests(unittest.TestCase):
    def test_ik_reuse_requires_same_recent_stopped_geometry(self):
        cache={'time':10.,'base':(0.,0.,.2),'target':(.25,0.),'world_yaw':0.}
        self.assertTrue(grasp_preflight_valid(cache,(.001,0.,.202),(.251,0.),0.,10.2))
        for pose,target,yaw,now in [((.01,0.,.2),(.25,0.),0.,10.2),
                                    ((0.,0.,.22),(.25,0.),0.,10.2),
                                    ((0.,0.,.2),(.26,0.),0.,10.2),
                                    ((0.,0.,.2),(.25,0.),1.,10.2),
                                    ((0.,0.,.2),(.25,0.),0.,13.)]:
            self.assertFalse(grasp_preflight_valid(cache,pose,target,yaw,now))
    def test_braking_envelope_holds_and_increases_with_gap(self):
        for u in (0.,.3,.6):
            previous=0.
            for gap in (.1,.25,.5,1.,2.,4.):
                v=braking_speed(gap,u)
                self.assertGreaterEqual(v,previous);previous=v
                if v>0:
                    travel=v*v/(2*1.2)+(v+u)*.25+u*v/1.2
                    self.assertLessEqual(travel+.25,gap+1e-9)
                self.assertLessEqual(v,braking_speed(gap,0.))
    def test_corridor_ignores_off_path_wall_but_keeps_front_obstacle(self):
        scan=self.scan(float('inf'),float('inf'))
        scan.ranges[195]=1.5 # wall lateral .388 m, outside swept corridor
        self.assertGreater(front_corridor_distance(scan),20.)
        scan.ranges[180]=1.2
        self.assertAlmostEqual(front_corridor_distance(scan),1.2)
    def test_missing_lidar_is_not_reported_as_clear(self):
        scan=self.scan(float('nan'),float('nan'))
        self.assertIsNone(front_corridor_distance(scan))
    def test_centimetre_final_docking_restores_last_packed_landing(self):
        pose=(4.85586,-1.96619,-1.64082);target=(4.84,-2.,-math.pi/2)
        occupied=[(5.2832,-2.4071),(4.68094,-2.23291)]
        point=(4.84,-2.24)
        self.assertNotIn(point,reachable_drop_candidates((5.,-2.4),point,pose,occupied))
        dx,dy=target[0]-pose[0],target[1]-pose[1]
        forward=math.cos(pose[2])*dx+math.sin(pose[2])*dy
        refined=(pose[0]+forward*math.cos(pose[2]),pose[1]+forward*math.sin(pose[2]),pose[2])
        self.assertIn(point,reachable_drop_candidates((5.,-2.4),point,refined,occupied))
    def test_gripper_world_axis_clears_square_without_rotating_the_base(self):
        for yaw in (0.,.35,1.2,2.7568,-2.8,math.pi/4):
            desired=square_gripper_yaw(yaw)
            x,y,z,w=downward_gripper_quaternion(yaw,desired)
            rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                               [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                               [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            c,s=math.cos(yaw),math.sin(yaw)
            world=np.array([[c,-s,0],[s,c,0],[0,0,1]])@rotation
            self.assertLessEqual(.05*(abs(world[0,0])+abs(world[1,0])),.050001)
            self.assertAlmostEqual(world[2,2],-1.)
            self.assertLessEqual(abs((yaw-desired+math.pi)%(2*math.pi)-math.pi),math.pi/4+1e-9)
    def test_old_diagonal_gripper_exceeds_65mm_at_observed_failure_heading(self):
        yaw=2.7568
        self.assertGreater(.05*(abs(math.cos(yaw))+abs(math.sin(yaw))),.065)
    def scan(self,front,rear):
        from types import SimpleNamespace
        ranges=[rear]*361
        for i in range(160,201):ranges[i]=front
        return SimpleNamespace(ranges=ranges,angle_min=-math.pi,angle_increment=math.pi/180,
                               range_min=.1,range_max=30.)
    def test_front_lane_does_not_use_rear_lidar_obstacles(self):
        self.assertTrue(lidar_front_lane_clear(self.scan(5.,.5),1.))
        self.assertFalse(lidar_front_lane_clear(self.scan(.5,5.),1.))
    def test_front_lane_includes_sensor_to_control_axle_offset(self):
        self.assertFalse(lidar_front_lane_clear(self.scan(1.25,5.),1.))
        self.assertTrue(lidar_front_lane_clear(self.scan(1.35,5.),1.))
    def test_drop_region_adapts_to_arrival_heading_without_moving_base(self):
        for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
            candidates=reachable_drop_candidates((5.,-2.4),(5.,-2.4),(4.77,-2.4,yaw),[])
            self.assertTrue(candidates,msg=str(yaw))
            for x,y in candidates:
                self.assertLessEqual(abs(x-5.),.40)
                self.assertLessEqual(abs(y+2.4),.17)
    def test_drop_candidates_protect_existing_cargo(self):
        occupied=[(5.,-2.4),(5.09,-2.32)]
        candidates=reachable_drop_candidates((5.,-2.4),(5.,-2.4),(4.77,-2.4,0.),occupied)
        self.assertTrue(candidates)
        self.assertTrue(all(math.dist(point,old)>=.12 for point in candidates for old in occupied))
    def test_drop_region_rejects_distant_unreachable_pose(self):
        self.assertEqual(reachable_drop_candidates((5.,-2.4),(5.,-2.4),(0.,0.,0.),[]),[])
    def test_departure_does_not_reverse_into_rear_or_rotated_cargo(self):
        self.assertEqual(departure_direction((0.,0.,0.),(.3,0.)),-1)
        self.assertEqual(departure_direction((0.,0.,0.),(-.3,0.)),1)
        self.assertEqual(departure_direction((0.,0.,math.pi/2),(0.,-.3)),1)
    def test_new_docking_side_avoids_earlier_deliveries(self):
        occupied=[(1.961,-6.033),(1.923,-5.914)]
        options=delivery_region_approaches((2.,-6.),occupied)
        self.assertTrue(options)
        for landing,base in options:
            self.assertTrue(all(math.dist(landing,p)>=.12 for p in occupied))
            c,s=math.cos(base[2]),math.sin(base[2])
            for x,y in occupied:
                dx,dy=x-base[0],y-base[1]
                self.assertGreaterEqual(footprint_clearance((c*dx+s*dy,-s*dx+c*dy)),.10)
    def test_all_five_region_landings_have_a_free_docking_side(self):
        occupied=[]
        for _ in range(5):
            options=delivery_region_approaches((0.,0.),occupied)
            self.assertTrue(options)
            landing,base=min(options,key=lambda item:math.dist(item[0],(0.,0.)))
            occupied.append(landing)
    def test_reachable_landing_is_rejected_when_departure_enters_a_wall(self):
        class Grid:
            def safe(self,pose):return pose[1]<.1
            def direct_base_path_clear(self,a,b,min_clearance):return b[1]<.1
        # Facing toward the region, backing out would cross the north wall.
        self.assertTrue(reachable_drop_candidates((0.,-.24),(0.,-.24),(0.,0.,-math.pi/2),[]))
        self.assertEqual(usable_drop_candidates(Grid(),(0.,-.24),(0.,-.24),(0.,0.,-math.pi/2),[]),[])
    def test_rear_reach_is_not_used_as_an_early_navigation_finish(self):
        class Grid:
            def safe(self,pose):return True
            def direct_base_path_clear(self,a,b,min_clearance):return True
        self.assertTrue(reachable_drop_candidates((-.5,0.),(-.5,0.),(0.,0.,0.),[]))
        self.assertEqual(usable_drop_candidates(Grid(),(-.5,0.),(-.5,0.),(0.,0.,0.),[]),[])
    def test_footprint_covers_original_body_and_wheels_with_margin(self):
        root=ET.parse(ROOT/'ros_ws/src/yzbot/mybot_description/urdf/large_base.xacro').getroot()
        props={e.attrib['name']:e.attrib['value'] for e in root if e.tag.endswith('property')}
        length=float(props['base_length']);width=float(props['base_width']);axle=float(props['wheel_joint_x'])
        radius=float(props['wheel_radius']);outer=width/2+float(props['wheel_length'])
        contour=np.asarray(NAV_FOOTPRINT,dtype=np.float32)
        points=[(x,y) for x in (-axle-length/2,-axle+length/2) for y in (-width/2,width/2)]
        points += [(x,y) for x in (-radius,radius) for y in (-outer,outer)]
        points += [(x,y) for x in (.05,.15) for y in (-.05,.05)]
        for p in points:self.assertGreaterEqual(cv2.pointPolygonTest(contour,p,True),.009)

    def test_rear_corner_does_not_include_nonexistent_wheel_width(self):
        yaw=-.81;c,s=math.cos(yaw),math.sin(yaw)
        old=[(-.33,-.23),(.15,-.23),(.15,.23),(-.33,.23)]
        left=lambda polygon:min(c*x-s*y for x,y in polygon)
        self.assertLess(left(old),-.37)
        self.assertGreater(left(NAV_FOOTPRINT),-.36)

    def test_camera_can_track_cube_in_front_of_actual_bumper(self):
        self.assertTrue(point_in_footprint(.12,0.))
        self.assertFalse(point_in_footprint(.22,0.))
        self.assertFalse(point_in_footprint(.12,.22))

if __name__=='__main__':unittest.main()
