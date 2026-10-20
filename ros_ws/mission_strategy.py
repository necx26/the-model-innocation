"""Map-only search/routing and image-only object memory (no simulator entities)."""
import heapq
import math
import threading
import time
from functools import lru_cache
from pathlib import Path

import cv2
import numpy as np
import yaml
from scene_config import SCENE, ZONES, PATROLS, in_delivery_area
CRUISE_SPEED = float(SCENE.get("cruise_speed_mps", .65))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def crop_occupancy_map(image, metadata, bounds):
    """Crop using this map's origin/resolution, with image bounds checked."""
    if image is None or image.size == 0:
        raise ValueError('导航地图图像不可用')
    xmin, xmax, ymin, ymax = bounds
    resolution = float(metadata['resolution'])
    ox, oy = metadata['origin'][:2]
    height, width = image.shape[:2]
    left = max(0, int(math.floor((xmin-ox)/resolution)))
    right = min(width, int(math.ceil((xmax-ox)/resolution))+1)
    top = max(0, height-1-int(math.ceil((ymax-oy)/resolution)))
    bottom = min(height, height-int(math.floor((ymin-oy)/resolution)))
    if right <= left or bottom <= top:
        raise ValueError('搜索范围与导航地图没有交集')
    return image[top:bottom, left:right].copy()


class MissionMap:
    def __init__(self, yaml_path, bounds=None):
        bounds = SCENE.get('search_bounds', (-7., 6.5, -5.5, 6.)) if bounds is None else bounds
        meta = yaml.safe_load(Path(yaml_path).read_text())
        image = cv2.imread(str(Path(yaml_path).parent / meta['image']), 0)
        self.display_map = crop_occupancy_map(image, meta, bounds)
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
        # The competition randomizer samples only floor points where the
        # gripper has a valid approach. Rank views by those legal spawn points
        # instead of spending equal search effort on unsampleable floor.
        self.spawnable = np.array([
            bool(self.approaches({'x': float(x), 'y': float(y)}))
            and not (-.2 < x < 2.4 and -1.9 < y < 1.7)
            for x, y in self.search_points
        ], dtype=bool)
        for r in range(4, len(ys) - 4, 11):
            for c in range(4, len(xs) - 4, 11):
                if self.walk[r, c] and self.clearance[r, c] >= .50:
                    p = self.xy((r, c))
                    visible = self.visible(p, None)
                    if len(visible):
                        delta=self.search_points[visible]-np.asarray(p)
                        self.views.append({'point': p, 'visible': visible,
                                           'bearings':np.arctan2(delta[:,1],delta[:,0])})
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

    def direct_base_path_clear(self, a, b, min_clearance=.38):
        """Require the full robot footprint to clear a straight approach lane."""
        length = math.dist(a[:2], b[:2])
        for t in np.linspace(0., 1., max(2, int(length / .05))):
            p=(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t)
            rc=self.cell(p)
            # `walk` is the general Nav2 route mask (0.43 m margin). A direct
            # target approach may use the true 0.38 m circumscribed footprint;
            # keeping the larger global margin here made clear lanes detour.
            if not self.inside(rc) or not self.free[rc] or self.clearance[rc] < min_clearance:
                return False
        return True

    def visible(self, point, yaw, radius=3.5):
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
        # The 65 mm open gripper cannot clear a 50 mm cube diagonally
        # (the projected width reaches 70.7 mm). Approach along cube axes.
        for yaw in np.arange(0., 2 * math.pi, math.pi / 2):
            # Stop with a visible gap so the camera does not lose the cube
            # under the chassis. The arm reaches the remaining distance.
            pose = (x - .22 * math.cos(yaw), y - .22 * math.sin(yaw), float(wrap(yaw)))
            if self.safe(pose) and self.line_free(pose, (x, y)):
                result.append({'grasp': pose, 'pregrasp': pose, 'view': pose})
        return result

    def direct_approach(self, target, current):
        """Prefer the visible target's current-side approach when its lane is clear."""
        if current is None or len(current)<3:return None
        x,y=target['x'],target['y']
        distance=math.hypot(x-current[0],y-current[1])
        if distance<.30:return None
        yaw=math.atan2(y-current[1],x-current[0])
        grasp=(x-.22*math.cos(yaw),y-.22*math.sin(yaw),float(wrap(yaw)))
        if not self.safe(grasp):return None
        if not self.direct_base_path_clear(current,grasp):return None
        if not self.line_free(grasp,(x,y)):return None
        return {'grasp':grasp,'pregrasp':grasp,'view':grasp,'direct_from_current':True}

    def next_view(self, current, seen, attempted, delivery=None, directional=False):
        field = self.distances(current)
        back = self.distances(delivery) if delivery is not None else None
        best = None
        for i, view in enumerate(self.views):
            if attempted.get(i, 0) >= 2:
                continue
            visible = view['visible']
            legal = visible[self.spawnable[visible]]
            unknown = legal[~seen[legal]]
            if not len(unknown):
                continue
            distance = self.distance(field, view['point'])
            service_cost = .5 * self.distance(back, view['point']) / CRUISE_SPEED if back is not None else 0.
            if not directional:
                gain=len(unknown)
                score=gain/(5.+distance/CRUISE_SPEED+service_cost+4.*attempted.get(i,0))
                if math.isfinite(distance) and (best is None or score>best[0]):
                    centre=np.mean(self.search_points[unknown],axis=0)
                    yaw=math.atan2(centre[1]-view['point'][1],centre[0]-view['point'][0])
                    best=(score,i,(*view['point'],yaw),gain,distance)
                continue
            # A front camera cannot see the full 360-degree visibility set.
            # Select a useful arrival heading and charge for both turns; the
            # real image observer remains the only source of negative evidence.
            travel_heading=math.atan2(view['point'][1]-current[1],view['point'][0]-current[0])
            bearings=np.arctan2(self.search_points[unknown,1]-view['point'][1],
                               self.search_points[unknown,0]-view['point'][0])
            headings=np.append(np.arange(-math.pi,math.pi,math.pi/8),travel_heading)
            heading_best=None
            initial_turn=abs(wrap(travel_heading-current[2])) if len(current)>2 else 0.
            for yaw in headings:
                errors=(bearings-yaw+math.pi)%(2*math.pi)-math.pi
                gain=int(np.count_nonzero(np.abs(errors)<.58))
                if not gain:continue
                turn=(initial_turn+abs(wrap(float(yaw)-travel_heading)))/.9
                score=gain/(2.+distance/CRUISE_SPEED+service_cost+turn+4.*attempted.get(i,0))
                if heading_best is None or score>heading_best[0]:heading_best=(score,float(yaw),gain)
            if heading_best is None:continue
            score,yaw,gain=heading_best
            if math.isfinite(distance) and (best is None or score > best[0]):
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

    def easy_visible_layout(self, seed=20261019):
        """Deterministic, camera-visible practice layout; never used by the controller."""
        import random
        rng = random.Random(int(seed))
        from_start = self.distances((0., 0.))
        feasible = []
        for view_index, view in enumerate(self.views):
            legal = view['visible'][self.spawnable[view['visible']]]
            if len(legal) < 10:
                continue
            points = self.search_points[legal]
            legal = legal[np.linalg.norm(points - np.asarray(view['point']), axis=1) <= 2.5]
            points = self.search_points[legal]
            order = list(range(len(legal)))
            random.Random(int(seed) ^ (view_index * 0x9E3779B1)).shuffle(order)
            selected = []
            for k in order:
                if all(math.dist(points[k], points[j]) > .72 for j in selected):
                    selected.append(k)
                if len(selected) == 10:
                    break
            if len(selected) != 10:
                continue
            route_cost = self.distance(from_start, view['point'])
            if math.isfinite(route_cost):
                feasible.append((route_cost, -len(legal), view_index, legal[selected]))
        if not feasible:
            raise ValueError('地图没有能从同一相机视野覆盖十个安全摆放点的易练习区域')
        _, _, view_index, chosen = min(feasible)
        positions = [tuple(self.search_points[int(i)]) for i in chosen]
        rng.shuffle(positions)
        colors = ['red'] * 5 + ['blue'] * 5
        rng.shuffle(colors)
        counts = {'red': 0, 'blue': 0}
        layout = []
        for color, (x, y) in zip(colors, positions):
            counts[color] += 1
            layout.append({'model': f'{color}_cube_{counts[color]}', 'color': color,
                           'sector': 0, 'x': float(x), 'y': float(y), 'z': .025})
        return {'seed': int(seed), 'layout': layout, 'sector_anchors': [], 'bounds': self.bounds,
                'minimum_spacing_m': .72, 'source': 'single_camera_view_practice_sampler',
                'sector_count': 1, 'visible_from_view_index': view_index}


class ObjectMemory:
    def __init__(self):
        self.lock = threading.RLock()
        self.items = []

    def update(self, detections):
        confirmed = []
        now = time.monotonic()
        with self.lock:
            # Match each existing object at most once per camera frame. Nearby
            # fragments of the same mask must not count as multiple confirmations.
            matched = set()
            for d in detections:
                if d['color'] not in ('red', 'blue') or not all(math.isfinite(d[k]) for k in ('x', 'y')):
                    continue
                # Ground projection can shift by 20-30 cm as a cube moves from
                # the edge of the RGB-D view toward the center. Keep one track
                # for that same visible cube instead of creating a phantom
                # second target. Competition cube placements are spaced well
                # beyond this association radius.
                options = [t for t in self.items if t['color'] == d['color'] and t['status'] in ('available','unreachable')
                           and math.hypot(t['x'] - d['x'], t['y'] - d['y']) < .32]
                if options:
                    t = min(options, key=lambda t: math.hypot(t['x'] - d['x'], t['y'] - d['y']))
                    if t['id'] in matched or t['status']=='unreachable': continue
                    t['samples'].append((d['x'], d['y'])); t['samples'] = t['samples'][-9:]
                    t['hits'] += 1
                    t['last_seen'] = now
                    t['x'], t['y'] = map(float, np.median(t['samples'], axis=0))
                    if t['hits'] == 2: confirmed.append(t.copy())
                else:
                    t = {'id': len(self.items) + 1, 'color': d['color'], 'x': d['x'], 'y': d['y'],
                         'hits': 1, 'samples': [(d['x'], d['y'])], 'status': 'available', 'failures': 0,
                         'last_seen': now, 'retry_after': 0.}
                    self.items.append(t)
                matched.add(t['id'])
        return confirmed

    def available(self, color=None):
        now = time.monotonic()
        with self.lock:
            return [t.copy() for t in self.items if t['hits'] >= 2 and t['status'] == 'available'
                    and t.get('retry_after', 0.) <= now
                    and (color is None or t['color'] == color)]

    def retry_delay(self, color):
        now = time.monotonic()
        with self.lock:
            delays = [t.get('retry_after', 0.) - now for t in self.items
                      if t['color'] == color and t['hits'] >= 2 and t['status'] == 'available'
                      and t.get('retry_after', 0.) > now]
        return min(delays, default=0.)

    def mark(self, ident, status):
        with self.lock:
            selected=next((t for t in self.items if t['id']==ident),None)
            for t in self.items:
                if t['id'] == ident:
                    t['status'] = status
            if status=='picked' and selected:
                # A cube displaced before collection can leave an old track.
                # Retire only stale nearby aliases; a fresh separate detection
                # stays valid, and two new images can confirm the old spot again.
                for t in self.items:
                    if (t['id']!=ident and t['status']=='available' and t['color']==selected['color']
                            and t['last_seen']<selected['last_seen']-1.
                            and math.hypot(t['x']-selected['x'],t['y']-selected['y'])<.40):
                        t['hits']=0;t['samples']=[];t['retry_after']=0.

    def fail(self, ident):
        with self.lock:
            for t in self.items:
                if t['id'] == ident:
                    t['failures'] += 1
                    t['retry_after'] = time.monotonic() + min(30., 8.*t['failures'])
                    if t['failures'] >= 3:t['status'] = 'unreachable'

    def snapshot(self):
        with self.lock:
            return [{k: v for k, v in t.items() if k != 'samples'} for t in self.items if t['hits'] >= 2]



def delivery_slot(zone, number):
    """Keep cargo on one x plane, with the chassis entirely behind that plane.

    Five 5 cm cubes fit into a 0.5 m deep delivery zone with 8 cm spacing.
    Increasing x on every drop made the next chassis goal intersect earlier cargo.
    """
    offsets = (0., -.08, .08, -.16, .16)
    if not 0 <= number < len(offsets):raise ValueError('每轮最多 5 个放置槽位')
    x,y = zone
    drop = (float(x), float(y+offsets[number]))
    return drop, (drop[0]-.23, drop[1], 0.)


def choose_observed_deliveries(grid,current,candidates,remaining,zones,start_number=0,
                               time_budget=None,time_scale=1.0):
    """A pessimistic ETA must not replace a visible legal target with blind search.

    Prefer the budget-feasible plan. If none fits, attempt the cheapest known
    next delivery (partial task credit remains possible), without changing order.
    """
    route=plan_deliveries(grid,current,candidates,remaining,zones,start_number,
                          time_budget=time_budget,time_scale=time_scale)
    if not route and remaining:
        route=plan_deliveries(grid,current,candidates,remaining[:1],zones,start_number,
                              time_scale=time_scale)
        for item in route:item['deadline_fallback']=True
    return route


def plan_deliveries(grid, current, candidates, remaining, zones, start_number=0, time_budget=None,
                    time_scale=1.0):
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
        _, base = delivery_slot(zones[color], start_number+offset)
        ends.append(base)
    sources = [current] + ends[:-1]
    costs = {}
    # Online calibration raises the map-only estimate when completed deliveries
    # consistently take longer than the nominal route and handling model.
    time_scale = max(1.0, min(2.25, float(time_scale)))
    for slot, color in enumerate(slots):
        df = grid.distances(sources[slot]); ef = grid.distances(ends[slot])
        for i, t in enumerate(candidates):
            if t['color'] != color:continue
            options = []
            approaches=list(grid.approaches(t))
            direct_approach=getattr(grid,'direct_approach',None)
            direct=direct_approach(t,sources[slot]) if callable(direct_approach) else None
            if direct is not None:approaches.insert(0,direct)
            for a in approaches:
                # Once a target is known from the camera, drive to its grasp
                # pose directly. The distant view point is a search vantage,
                # not a required waypoint in the pick-up route.
                approach_pose=a.get('pregrasp',a['view'])
                if a.get('direct_from_current'):
                    travel=math.dist(sources[slot][:2],approach_pose[:2])
                    initial_heading=math.atan2(approach_pose[1]-sources[slot][1],
                                               approach_pose[0]-sources[slot][0])
                    turn=abs(wrap(initial_heading-sources[slot][2]))+abs(wrap(a['grasp'][2]))
                else:
                    travel=grid.distance(df,approach_pose)
                    turn=abs(wrap(a['grasp'][2]-sources[slot][2]))+abs(wrap(a['grasp'][2]))
                cost=travel+grid.distance(ef,a['grasp'])+.5
                options.append(((cost/CRUISE_SPEED+turn/.9+17.+6.*t.get('failures',0))*time_scale,a))
            if options:
                best = min(options, key=lambda v: v[0])
                if math.isfinite(best[0]): costs[slot, i] = best

    @lru_cache(None)
    def solve(slot, mask, limit):
        if slot == limit: return 0, 0., ()
        # Maximize the feasible prefix, then minimize its cost. An unreachable
        # later object must not block delivery of an already reachable object.
        best = (0, 0., ())
        for i, t in enumerate(candidates):
            if mask & (1 << i) or (slot, i) not in costs: continue
            length, rest, order = solve(slot + 1, mask | (1 << i), limit)
            candidate = (length+1, costs[slot, i][0]+rest, (i,)+order)
            if candidate[0] > best[0] or (candidate[0] == best[0] and candidate[1] < best[1]):
                best = candidate
        return best
    # Choose the longest color-ordered prefix whose minimum known route fits
    # the remaining exam time; this is exact for the observed candidate set.
    length,total,order=0,0.,()
    for limit in range(len(slots),0,-1):
        candidate_length,candidate_total,candidate_order=solve(0,0,limit)
        if time_budget is None or candidate_total<=time_budget:
            length,total,order=candidate_length,candidate_total,candidate_order
            break
    if not length: return []
    return [{'target': candidates[i], **costs[slot, i][1], 'drop': ends[slot],
             'estimated_seconds': round(costs[slot, i][0], 2), 'known_plan_seconds': round(total, 2)}
            for slot, i in enumerate(order)]
