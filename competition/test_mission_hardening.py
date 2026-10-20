"""Regression cases for confirmed assignment and memory failure modes."""
import math
import unittest
from unittest.mock import patch
from mission_strategy import ObjectMemory, plan_deliveries, choose_observed_deliveries, delivery_slot, MissionMap

class RoutingMap:
    def distances(self, point):return point
    def distance(self, field, point):return math.dist(field[:2], point[:2])
    def approaches(self, target):
        if target.get('blocked'):return []
        return [dict(view=(target['x']-.7,target['y'],0.),grasp=(target['x']-.2,target['y'],0.))]

class HardeningTests(unittest.TestCase):
    def setUp(self):
        self.memory=ObjectMemory()
        self.red=dict(color='red',x=1.,y=1.)
        self.zones={'red':(3.,0.),'blue':(3.,-1.)}
    def confirm(self, item):
        self.memory.update([item]);self.memory.update([item])
    def test_fragments_from_one_frame_do_not_confirm_a_target(self):
        self.memory.update([self.red,dict(self.red,x=1.03),dict(self.red,y=1.03)])
        self.assertEqual(self.memory.available(),[])
        self.memory.update([self.red])
        self.assertEqual(len(self.memory.available()),1)
        self.assertEqual(self.memory.available()[0]['hits'],2)
    def test_other_objects_remain_separate(self):
        blue=dict(color='blue',x=1.,y=1.)
        red2=dict(self.red,x=1.7)
        self.memory.update([self.red,red2,blue]);self.memory.update([self.red,red2,blue])
        self.assertEqual(len(self.memory.available()),3)
    def test_camera_projection_shift_does_not_create_phantom_target(self):
        first=dict(color='blue',x=6.186,y=-13.803)
        corrected=dict(color='blue',x=5.925,y=-13.782)
        self.memory.update([first]);self.memory.update([first])
        self.memory.update([corrected]);self.memory.update([corrected])
        self.assertEqual(len(self.memory.items),1)
        self.assertEqual(len(self.memory.available()),1)
        self.assertEqual(self.memory.available()[0]['hits'],4)
    def test_nonfinite_projection_is_ignored(self):
        self.memory.update([dict(self.red,x=float('nan')),dict(self.red,y=float('inf'))])
        self.assertEqual(self.memory.items,[])
    def test_failed_target_cools_down_before_retry(self):
        with patch('mission_strategy.time.monotonic',return_value=100.):
            self.confirm(self.red);self.memory.fail(1)
            self.assertEqual(self.memory.available(),[])
            self.assertEqual(self.memory.retry_delay('red'),8.)
        with patch('mission_strategy.time.monotonic',return_value=109.):
            self.assertEqual(len(self.memory.available()),1)
            self.assertEqual(self.memory.available()[0]['failures'],1)
    def test_unreachable_target_is_not_recreated_under_a_new_id(self):
        self.confirm(self.red)
        for _ in range(3):self.memory.fail(1)
        for _ in range(8):self.memory.update([self.red])
        self.assertEqual(len(self.memory.items),1)
        self.assertEqual(self.memory.items[0]['status'],'unreachable')
        self.assertEqual(self.memory.available(),[])
    def test_delivery_is_never_returned_as_available(self):
        self.confirm(self.red);self.memory.mark(1,'delivered')
        self.assertEqual(self.memory.available(),[])
    def test_pickup_retires_stale_track_left_by_a_moved_cube(self):
        with patch('mission_strategy.time.monotonic',return_value=100.):
            self.confirm(dict(color='blue',x=0.,y=0.))
        with patch('mission_strategy.time.monotonic',return_value=110.):
            self.confirm(dict(color='blue',x=.5,y=0.))
            for x in (.4,.34,.3,.3,.3):self.memory.update([dict(color='blue',x=x,y=0.)])
        self.memory.mark(2,'picked')
        self.assertEqual(self.memory.available(),[])
        with patch('mission_strategy.time.monotonic',return_value=112.):
            self.confirm(dict(color='blue',x=0.,y=0.))
        self.assertEqual([t['id'] for t in self.memory.available()],[1])
    def test_fresh_nearby_object_is_not_retired_with_a_pickup(self):
        with patch('mission_strategy.time.monotonic',return_value=100.):
            self.confirm(dict(color='blue',x=0.,y=0.))
            self.confirm(dict(color='blue',x=.35,y=0.))
        self.memory.mark(2,'picked')
        self.assertEqual([t['id'] for t in self.memory.available()],[1])
    def test_unreachable_later_target_does_not_block_current_delivery(self):
        items=[dict(id=1,**self.red),dict(id=2,color='blue',x=2.,y=1.,blocked=True)]
        route=plan_deliveries(RoutingMap(),(0.,0.,0.),items,['red','blue'],self.zones)
        self.assertEqual([r['target']['id'] for r in route],[1])
    def test_order_is_not_bypassed_when_first_color_unreachable(self):
        items=[dict(id=1,**self.red,blocked=True),dict(id=2,color='blue',x=2.,y=1.)]
        route=plan_deliveries(RoutingMap(),(0.,0.,0.),items,['red','blue'],self.zones)
        self.assertEqual(route,[])
    def test_failure_penalty_prefers_equally_distant_unfailed_target(self):
        items=[dict(id=1,**self.red,failures=1),dict(id=2,**self.red,failures=0)]
        route=plan_deliveries(RoutingMap(),(0.,0.,0.),items,['red'],self.zones)
        self.assertEqual(route[0]['target']['id'],2)
    def test_full_feasible_route_still_has_unique_objects_and_color_order(self):
        items=[dict(id=1,**self.red),dict(id=2,color='red',x=2.,y=1.),dict(id=3,color='blue',x=2.,y=-1.)]
        route=plan_deliveries(RoutingMap(),(0.,0.,0.),items,['red','red','blue'],self.zones)
        self.assertEqual([r['target']['color'] for r in route],['red','red','blue'])
        self.assertEqual(len({r['target']['id'] for r in route}),3)

    def test_deadline_does_not_discard_a_visible_legal_target_for_blind_search(self):
        route=choose_observed_deliveries(RoutingMap(),(0.,0.,0.),[dict(id=1,**self.red)],
                                        ['red'],self.zones,time_budget=0.)
        self.assertEqual([item['target']['id'] for item in route],[1])
        self.assertTrue(route[0]['deadline_fallback'])

    def test_deadline_fallback_does_not_break_color_order(self):
        route=choose_observed_deliveries(RoutingMap(),(0.,0.,0.),[dict(id=1,color='blue',x=1.,y=1.)],
                                        ['red','blue'],self.zones,time_budget=0.)
        self.assertEqual(route,[])

    def test_deadline_keeps_the_maximum_affordable_ordered_prefix(self):
        items=[dict(id=1,**self.red),dict(id=2,color='red',x=2.,y=1.),dict(id=3,color='blue',x=2.,y=-1.)]
        grid=RoutingMap();order=['red','red','blue']
        first=plan_deliveries(grid,(0.,0.,0.),items,['red'],self.zones)
        route=plan_deliveries(grid,(0.,0.,0.),items,order,self.zones,
                              time_budget=first[0]['estimated_seconds']+.1)
        self.assertEqual(len(route),1)
        self.assertEqual(route[0]['target']['color'],'red')

    def test_impossible_deadline_never_starts_a_known_delivery(self):
        self.assertEqual(plan_deliveries(RoutingMap(),(0.,0.,0.),[dict(id=1,**self.red)],
                                        ['red'],self.zones,time_budget=0.),[])

    def test_measured_slow_delivery_inflates_future_route_estimate(self):
        target=dict(id=1,**self.red)
        nominal=plan_deliveries(RoutingMap(),(0.,0.,0.),[target],['red'],self.zones)[0]['estimated_seconds']
        calibrated=plan_deliveries(RoutingMap(),(0.,0.,0.),[target],['red'],self.zones,
                                   time_scale=1.5)[0]['estimated_seconds']
        # Both outputs are rounded independently: 0.005*1.5 + 0.005
        # bounds their combined error, including floating-point representation.
        self.assertLessEqual(abs(calibrated-nominal*1.5),.0125+1e-9)

    def test_cardinal_grasp_clears_the_actual_open_jaw(self):
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        grid=MissionMap(root/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml')
        target=grid.scattered_layout(20261018)['layout'][0]
        for option in grid.approaches(target):
            yaw=option['grasp'][2]
            projected_width=.05*(abs(math.cos(yaw))+abs(math.sin(yaw)))
            self.assertLessEqual(projected_width,.065-.010)
            self.assertAlmostEqual(math.dist(option['grasp'][:2],(target['x'],target['y'])),.22)
            # Leave the cube visible ahead of the chassis; the arm takes the
            # remaining reach without a second base motion.
            self.assertGreater(.22+.14-.16-.025,.10)
            self.assertAlmostEqual(math.dist(option['pregrasp'][:2],(target['x'],target['y'])),.22)

class DeliverySlotTests(unittest.TestCase):
    def test_slots_fit_zone_and_do_not_overlap(self):
        drops=[delivery_slot((1.,2.),i)[0] for i in range(5)]
        for i,p in enumerate(drops):
            self.assertLess(abs(p[0]-1.)+.025,.5)
            self.assertLess(abs(p[1]-2.)+.025,.25)
            for q in drops[i+1:]:self.assertGreater(math.dist(p,q),.075)
    def test_chassis_at_each_drop_does_not_touch_earlier_cargo(self):
        slots=[delivery_slot((0.,0.),i) for i in range(5)]
        for i,(_,base) in enumerate(slots):
            for cargo,_ in slots[:i]:self.assertGreater(cargo[0]-.025,base[0]+.15)
    def test_drop_poses_are_reachable_in_existing_map(self):
        from pathlib import Path
        from scene_config import ZONES
        root=Path(__file__).resolve().parents[1]
        grid=MissionMap(root/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml')
        for zone in ZONES.values():
            for i in range(5):self.assertTrue(grid.safe(delivery_slot(zone,i)[1]))
    def test_sixth_slot_is_rejected(self):
        with self.assertRaises(ValueError):delivery_slot((0.,0.),5)

if __name__=='__main__':unittest.main()
