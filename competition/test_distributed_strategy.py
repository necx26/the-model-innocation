"""Safety/mission invariants for the distributed-office strategy."""
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'ros_ws'))
from mission_strategy import MissionMap, ObjectMemory, plan_deliveries


class StrategyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = MissionMap(ROOT/'ros_ws/src/yzbot/bot_navigation/maps/map.yaml')

    def test_barrier_requires_a_detour(self):
        a, b = (0., 2.8), (0., 4.3)
        self.assertFalse(self.grid.line_free(a, b))
        cost = self.grid.distance(self.grid.distances(a), b)
        self.assertTrue(math.isfinite(cost))
        self.assertGreater(cost, math.dist(a, b) + 1.)

    def test_repeatable_scatter_has_seven_regions_and_clear_grasps(self):
        data = self.grid.scattered_layout(20261018)
        self.assertEqual(data, self.grid.scattered_layout(20261018))
        items = data['layout']
        self.assertEqual(len(set(t['sector'] for t in items)), 7)
        self.assertEqual(sum(t['color'] == 'red' for t in items), 5)
        self.assertEqual(sum(t['color'] == 'blue' for t in items), 5)
        for i, t in enumerate(items):
            self.assertTrue(self.grid.approaches(t))
            for u in items[i+1:]:
                self.assertGreater(math.hypot(t['x']-u['x'], t['y']-u['y']), .65)

    def test_hidden_targets_do_not_enter_memory_or_routes(self):
        memory = ObjectMemory()
        self.assertEqual(plan_deliveries(self.grid, (0, 0, 0), memory.available(),
                                        ['red'], {'red': (3, 0), 'blue': (3, -1)}), [])
        d = {'color': 'red', 'x': -2.5, 'y': .9}
        memory.update([d]);self.assertEqual(memory.available(), [])
        memory.update([d]);self.assertEqual(len(memory.available()), 1)
        self.assertNotIn('model', memory.available()[0])
        memory.mark(1, 'delivered');self.assertEqual(memory.available(), [])

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
