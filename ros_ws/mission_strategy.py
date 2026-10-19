"""Map-only search/routing and image-only object memory (no simulator entities)."""
import heapq
import math
import threading
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import yaml
from scene_config import SCENE, ZONES, PATROLS, in_delivery_area
CRUISE_SPEED = float(SCENE.get("cruise_speed_mps", .65))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MissionMap:
    def __init__(self, yaml_path, bounds=(-7., 6.5, -5.5, 6.)):
        meta = yaml.safe_load(Path(yaml_path).read_text())
        image = cv2.imread(str(Path(yaml_path).parent / meta['image']), 0)
        self.res = .10
        self.bounds = tuple(bounds)
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        xs = np.arange(self.xmin, self.xmax + .001, self.res)
        ys = np.arange(self.ymax, self.ymin - .001, -self.res)
        cols = np.rint((xs - meta['origin'][0]) / meta['resolution']).astype(int)
        rows = image.shape[0] - 1 - np.rint((ys - meta['origin'][1]) / meta['resolution']).astype(int)
        self.free = image[np.ix_(rows, cols)] > 250
        self.clearance = cv2.distanceTransform(self.free.astype('uint8'), cv2.DIST_L2, 5) * self.res
        walk = (self.clearance >= .43).astype('uint8')
        _, labels = cv2.connectedComponents(walk, connectivity=8)
        origin = self.cell((0., 0.))
        self.walk = labels == labels[origin]
        self.walk &= walk > 0
        self.weight = 1. + .35 * np.exp(-np.maximum(self.clearance - .43, 0) / .25)
        # Same predictive patrol preference as Nav2; live obstacles remain Nav2's job.
        for patrol in PATROLS:
            a,b=patrol['start'],patrol['end']
            risk = (xs[None,:]>min(a[0],b[0])-.65)&(xs[None,:]<max(a[0],b[0])+.65)&(ys[:,None]>min(a[1],b[1])-.65)&(ys[:,None]<max(a[1],b[1])+.65)
            self.weight[risk] += 1.5
        self._fields = {}
        self.search_points = []
        self.views = []
        for r in range(3, len(ys) - 3, 6):
            for c in range(3, len(xs) - 3, 6):
                x, y = self.xy((r, c))
                # Delivery area and initial platform are not cargo search areas.
                excluded = in_delivery_area((x,y), extra=.25) or math.hypot(x, y) < .85
                if self.walk[r, c] and self.clearance[r, c] >= .63 and not excluded:
                    self.search_points.append((x, y))
        self.search_points = np.array(self.search_points, dtype=float)
        for r in range(4, len(ys) - 4, 11):
            for c in range(4, len(xs) - 4, 11):
                if self.walk[r, c] and self.clearance[r, c] >= .50:
                    p = self.xy((r, c))
                    visible = self.visible(p, None)
                    if len(visible):
                        self.views.append({'point': p, 'visible': visible})

    def cell(self, p):
        return (int(round((self.ymax - p[1]) / self.res)), int(round((p[0] - self.xmin) / self.res)))

    def xy(self, rc):
        return (round(self.xmin + rc[1] * self.res, 4), round(self.ymax - rc[0] * self.res, 4))

    def inside(self, rc):
        return 0 <= rc[0] < self.free.shape[0] and 0 <= rc[1] < self.free.shape[1]

    def safe(self, p, margin=.43):
        rc = self.cell(p)
        return self.inside(rc) and self.walk[rc] and self.clearance[rc] >= margin

    def snap(self, p):
        rc = self.cell(p)
        if self.inside(rc) and self.walk[rc]:
            return rc
        good = np.argwhere(self.walk)
        ds = (good[:, 0] - rc[0]) ** 2 + (good[:, 1] - rc[1]) ** 2
        k = int(np.argmin(ds))
        if ds[k] > 36:
            return None
        return tuple(good[k])

    def distances(self, p):
        src = self.snap(p)
        if src is None:
            return np.full(self.walk.shape, np.inf)
        if src in self._fields:
            return self._fields[src]
        ds = np.full(self.walk.shape, np.inf)
        ds[src] = 0.
        queue = [(0., src[0], src[1])]
        h, w = self.walk.shape
        steps = [(-1, 0, 1.), (1, 0, 1.), (0, -1, 1.), (0, 1, 1.),
                 (-1, -1, 1.414214), (-1, 1, 1.414214), (1, -1, 1.414214), (1, 1, 1.414214)]
        while queue:
            cost, r, c = heapq.heappop(queue)
            if cost > ds[r, c]:
                continue
            for dr, dc, step in steps:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < h and 0 <= nc < w and self.walk[nr, nc]):
                    continue
                if dr and dc and not (self.walk[r + dr, c] and self.walk[r, c + dc]):
                    continue
                value = cost + step * self.res * (self.weight[r, c] + self.weight[nr, nc]) / 2
                if value < ds[nr, nc]:
                    ds[nr, nc] = value
                    heapq.heappush(queue, (value, nr, nc))
        if len(self._fields) > 48:
            self._fields.clear()
        self._fields[src] = ds
        return ds

    def distance(self, field, p):
        rc = self.cell(p)
        return float(field[rc]) if self.inside(rc) and self.walk[rc] else math.inf

    def line_free(self, a, b):
        length = math.dist(a[:2], b[:2])
        for t in np.linspace(0, 1, max(2, int(length / .05))):
            rc = self.cell((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
            if not self.inside(rc) or not self.free[rc]:
                return False
        return True

    def visible(self, point, yaw, radius=1.7):
        if not len(self.search_points):
            return np.array([], dtype=int)
        delta = self.search_points - np.asarray(point[:2])
        dist = np.linalg.norm(delta, axis=1)
        mask = (dist < radius) & (dist > .35)
        if yaw is not None:
            err = (np.arctan2(delta[:, 1], delta[:, 0]) - yaw + math.pi) % (2 * math.pi) - math.pi
            mask &= np.abs(err) < .66
        return np.array([i for i in np.flatnonzero(mask) if self.line_free(point, self.search_points[i])], dtype=int)

    def approaches(self, target):
        x, y = target['x'], target['y']
        result = []
        for yaw in np.arange(0., 2 * math.pi, math.pi / 8):
            poses = [(x - d * math.cos(yaw), y - d * math.sin(yaw), float(wrap(yaw))) for d in [.20, .40, .70]]
            if all(self.safe(p) for p in poses) and self.line_free(poses[-1], (x, y)):
                result.append({'grasp': poses[0], 'view': poses[-1]})
        return result

    def next_view(self, current, seen, attempted, delivery=None):
        field = self.distances(current)
        back = self.distances(delivery) if delivery is not None else None
        best = None
        for i, view in enumerate(self.views):
            if attempted.get(i, 0) >= 2:
                continue
            gain = int(np.count_nonzero(~seen[view['visible']]))
            if gain == 0:
                continue
            distance = self.distance(field, view['point'])
            service_cost = .5 * self.distance(back, view['point']) / CRUISE_SPEED if back is not None else 0.
            score = gain / (5. + distance / CRUISE_SPEED + service_cost + 4. * attempted.get(i, 0))
            if math.isfinite(distance) and (best is None or score > best[0]):
                centre = np.mean(self.search_points[view['visible'][~seen[view['visible']]]], axis=0)
                yaw = math.atan2(centre[1] - view['point'][1], centre[0] - view['point'][0])
                best = (score, i, (*view['point'], yaw), gain, distance)
        return best

    def scattered_layout(self, seed, sectors=7):
        import random
        rng = random.Random(seed)
        pool = [tuple(p) for p in self.search_points if self.approaches({'x': p[0], 'y': p[1]})
                and not (-.2 < p[0] < 2.4 and -1.9 < p[1] < 1.7)]
        points = np.array(pool)
        # Geodesic farthest-point partitions spread objects behind different barriers.
        anchors = [pool[min(range(len(pool)), key=lambda k: math.dist(pool[k], (0., 0.)))]]
        fields = []
        nearest = np.full(len(pool), np.inf)
        for _ in range(sectors):
            f = self.distances(anchors[-1]); fields.append(f)
            nearest = np.minimum(nearest, [self.distance(f, p) for p in pool])
            anchors.append(pool[int(np.argmax(np.where(np.isfinite(nearest), nearest, -1)))])
        anchors = anchors[:sectors]
        assignments = np.argmin([[self.distance(f, p) for p in pool] for f in fields], axis=0)
        colors = ['red'] * 5 + ['blue'] * 5; rng.shuffle(colors)
        layout = []
        for trial in range(64):
            order=list(range(sectors));rng.shuffle(order)
            layout=[];repeated=set()
            def choices_for(sector):
                return [tuple(points[k]) for k in np.flatnonzero(assignments==sector)
                        if all(math.dist(points[k],(o['x'],o['y']))>.65 for o in layout)]
            for i,color in enumerate(colors):
                if i<sectors:
                    sector=order[i];choices=choices_for(sector)
                else:
                    options=[(k,choices_for(k)) for k in range(sectors) if k not in repeated]
                    options=[(k,ps) for k,ps in options if ps]
                    if not options:break
                    sector,choices=rng.choice(options);repeated.add(sector)
                if not choices:break
                p=rng.choice(choices)
                number=1+sum(o['color']==color for o in layout)
                layout.append({'model':f'{color}_cube_{number}','color':color,'sector':int(sector),
                               'x':float(p[0]),'y':float(p[1]),'z':.025})
            if len(layout)==10:break
        else:raise ValueError('七个分区内没有足够的可抓取摆放空间，请检查地图/放置区')
        return {'seed': seed, 'layout': layout, 'sector_anchors': anchors, 'bounds': self.bounds,
                'minimum_spacing_m': .65, 'source': 'map_safe_geodesic_partitions', 'sector_count': sectors}


class ObjectMemory:
    def __init__(self):
        self.lock = threading.RLock()
        self.items = []

    def update(self, detections):
        confirmed = []
        with self.lock:
            for d in detections:
                options = [t for t in self.items if t['color'] == d['color'] and t['status'] == 'available'
                           and math.hypot(t['x'] - d['x'], t['y'] - d['y']) < .18]
                if options:
                    t = min(options, key=lambda t: math.hypot(t['x'] - d['x'], t['y'] - d['y']))
                    t['samples'].append((d['x'], d['y']));t['samples'] = t['samples'][-9:]
                    t['hits'] += 1
                    t['x'], t['y'] = map(float, np.median(t['samples'], axis=0))
                    if t['hits'] == 2: confirmed.append(t.copy())
                else:
                    self.items.append({'id': len(self.items) + 1, 'color': d['color'], 'x': d['x'], 'y': d['y'],
                                       'hits': 1, 'samples': [(d['x'], d['y'])], 'status': 'available', 'failures': 0})
        return confirmed

    def available(self, color=None):
        with self.lock:
            return [t.copy() for t in self.items if t['hits'] >= 2 and t['status'] == 'available'
                    and (color is None or t['color'] == color)]

    def mark(self, ident, status):
        with self.lock:
            for t in self.items:
                if t['id'] == ident:
                    t['status'] = status

    def fail(self, ident):
        with self.lock:
            for t in self.items:
                if t['id'] == ident:
                    t['failures'] += 1
                    if t['failures'] >= 2:t['status'] = 'unreachable'

    def snapshot(self):
        with self.lock:
            return [{k: v for k, v in t.items() if k != 'samples'} for t in self.items if t['hits'] >= 2]


def plan_deliveries(grid, current, candidates, remaining, zones, start_number=0):
    """Exact finite assignment over observed objects and valid approach angles.

    Single gripper: every action ends at its prescribed delivery slot. Static
    geodesic/risk cost is an estimate; unknown targets and live traffic are not
    claimed globally optimal.
    """
    slots = []
    available = {c: sum(t['color'] == c for t in candidates) for c in ['red', 'blue']}
    for color in remaining:
        if not available[color]: break
        available[color] -= 1
        slots.append(color)
    if not slots:return []
    ends = []
    for offset, color in enumerate(slots):
        zx, zy = zones[color]
        n = start_number + offset
        ends.append((zx + .1 * (n % 3) - .2, zy + .08 * (n // 3 - .5), 0.))
    sources = [current] + ends[:-1]
    costs = {}
    for slot, color in enumerate(slots):
        df = grid.distances(sources[slot]); ef = grid.distances(ends[slot])
        for i, t in enumerate(candidates):
            if t['color'] != color:continue
            options = []
            for a in grid.approaches(t):
                cost = grid.distance(df, a['view']) + grid.distance(ef, a['grasp']) + .5
                turn = abs(wrap(a['grasp'][2] - sources[slot][2])) + abs(wrap(a['grasp'][2]))
                options.append((cost / CRUISE_SPEED + turn / .9 + 17., a))
            if options:
                best = min(options, key=lambda v: v[0])
                if math.isfinite(best[0]): costs[slot, i] = best

    @lru_cache(None)
    def solve(slot, mask):
        if slot == len(slots):return 0., ()
        best = (math.inf, ())
        for i, t in enumerate(candidates):
            if mask & (1 << i) or (slot, i) not in costs:continue
            rest, order = solve(slot + 1, mask | (1 << i))
            value = costs[slot, i][0] + rest
            if value < best[0]:best = value, (i,) + order
        return best
    total, order = solve(0, 0)
    if not math.isfinite(total):return []
    return [{'target': candidates[i], **costs[slot, i][1], 'drop': ends[slot],
             'estimated_seconds': round(costs[slot, i][0], 2), 'known_plan_seconds': round(total, 2)}
            for slot, i in enumerate(order)]
