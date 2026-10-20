"""Safety/mission invariants for the distributed-office strategy."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros_ws'))
from mission_strategy import MissionMap, ObjectMemory, delivery_slot, plan_deliveries


class StrategyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = MissionMap(ROOT/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml')

    def test_barrier_requires_a_detour(self):
        from scene_config import ZONES
        a, b = ZONES['A'], ZONES['B']
        self.assertFalse(self.grid.line_free(a, b))
        cost = self.grid.distance(self.grid.distances(a), b)
        self.assertTrue(math.isfinite(cost))
        self.assertGreater(cost, math.dist(a, b) + 1.)

    def test_repeatable_scatter_has_seven_regions_and_clear_grasps(self):
        data = self.grid.scattered_layout(4)
        self.assertEqual(data, self.grid.scattered_layout(4))
        items = data['layout']
        self.assertEqual(len(set(t['sector'] for t in items)), 7)
        self.assertEqual(sum(t['color'] == 'red' for t in items), 5)
        self.assertEqual(sum(t['color'] == 'blue' for t in items), 5)
        for i, t in enumerate(items):
            self.assertTrue(self.grid.approaches(t))
            for u in items[i+1:]:
                self.assertGreater(math.hypot(t['x']-u['x'], t['y']-u['y']), .65)

    def test_layout_stress_matrix_covers_24_seeds(self):
        # Random layouts are unbounded, so exercise a reproducible seed matrix
        # that checks sector coverage, color counts, spacing, and graspability.
        for seed in range(1, 25):
            with self.subTest(seed=seed):
                items = self.grid.scattered_layout(seed)['layout']
                self.assertEqual(len(items), 10)
                self.assertEqual(len({t['sector'] for t in items}), 7)
                self.assertEqual(sum(t['color'] == 'red' for t in items), 5)
                self.assertEqual(sum(t['color'] == 'blue' for t in items), 5)
                for i, target in enumerate(items):
                    self.assertTrue(self.grid.approaches(target))
                    for other in items[i+1:]:
                        self.assertGreater(
                            math.hypot(target['x']-other['x'], target['y']-other['y']),
                            .65)

    def test_easy_practice_layout_is_repeatable_and_camera_visible(self):
        data = self.grid.easy_visible_layout(20261019)
        self.assertEqual(data, self.grid.easy_visible_layout(20261019))
        items = data['layout']
        self.assertEqual(len(items), 10)
        self.assertEqual(sum(t['color'] == 'red' for t in items), 5)
        self.assertEqual(sum(t['color'] == 'blue' for t in items), 5)
        view = self.grid.views[data['visible_from_view_index']]
        for i, target in enumerate(items):
            self.assertTrue(self.grid.approaches(target))
            point_index = int(np.argmin(np.linalg.norm(
                self.grid.search_points - np.array([target['x'], target['y']]), axis=1)))
            self.assertIn(point_index, view['visible'])
            self.assertLessEqual(math.dist((target['x'], target['y']), view['point']), 2.5)
            for other in items[i+1:]:
                self.assertGreater(math.hypot(target['x']-other['x'], target['y']-other['y']), .72)

    def test_five_delivery_slots_stay_inside_zone_with_cube_clearance(self):
        zone=(2.0,-6.0)
        slots=[delivery_slot(zone,i)[0] for i in range(5)]
        self.assertTrue(all(abs(x-zone[0])<.5 and abs(y-zone[1])<.25 for x,y in slots))
        ordered=sorted(y for _,y in slots)
        self.assertTrue(all(b-a>=.079 for a,b in zip(ordered,ordered[1:])))
        self.assertTrue(all(abs(delivery_slot(zone,i)[1][0]-(slots[i][0]-.23))<1e-9 for i in range(5)))

    def test_search_prior_matches_legal_randomizer_spawn_points(self):
        data = self.grid.scattered_layout(495676667)
        for item in data['layout']:
            d = np.linalg.norm(self.grid.search_points-[item['x'],item['y']],axis=1)
            i = int(np.argmin(d))
            self.assertLess(d[i],1e-6)
            self.assertTrue(self.grid.spawnable[i])
        view = self.grid.next_view((0.,0.,0.),np.ones(len(self.grid.search_points),dtype=bool),{})
        self.assertIsNone(view)

    def test_hidden_targets_do_not_enter_memory_or_routes(self):
        memory = ObjectMemory()
        self.assertEqual(plan_deliveries(self.grid, (0, 0, 0), memory.available(),
                                        ['red'], {'red': (3, 0), 'blue': (3, -1)}), [])
        d = {'color': 'red', 'x': -2.5, 'y': .9}
        memory.update([d]);self.assertEqual(memory.available(), [])
        memory.update([d]);self.assertEqual(len(memory.available()), 1)
        self.assertNotIn('model', memory.available()[0])
        memory.mark(1, 'delivered');self.assertEqual(memory.available(), [])

    def test_visible_target_prefers_a_clear_straight_approach_from_robot_side(self):
        found=None
        for target in self.grid.scattered_layout(4)['layout']:
            for option in self.grid.approaches(target):
                yaw=option['grasp'][2]
                current=(target['x']-2.*math.cos(yaw),target['y']-2.*math.sin(yaw),yaw)
                direct=self.grid.direct_approach(target,current)
                if direct is not None:
                    found=(target,current,direct);break
            if found:break
        self.assertIsNotNone(found)
        target,current,direct=found
        self.assertTrue(direct['direct_from_current'])
        self.assertTrue(self.grid.direct_base_path_clear(current,direct['pregrasp']))
        self.assertAlmostEqual(math.dist(direct['grasp'][:2],(target['x'],target['y'])),.22)
        self.assertLess(math.dist(current[:2],direct['pregrasp'][:2]),math.dist(current[:2],(target['x'],target['y'])))

    def test_route_preserves_color_order_and_single_use(self):
        items = self.grid.scattered_layout(20261018)['layout']
        candidates = [dict(id=i+1, color=t['color'], x=t['x'], y=t['y']) for i, t in enumerate(items)]
        order = ['red', 'red', 'blue', 'blue', 'blue']
        route = plan_deliveries(self.grid, (0, 0, 0), candidates, order,
                                {'red': (3, 0), 'blue': (3, -1)})
        self.assertEqual([s['target']['color'] for s in route], order)
        self.assertEqual(len(set(s['target']['id'] for s in route)), 5)
        self.assertTrue(all(self.grid.safe(s['grasp']) and self.grid.safe(s['view']) for s in route))


if __name__ == '__main__':
    unittest.main()
