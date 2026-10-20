"""Continuous swept-path checks for kinematic obstacle cars against SDF boxes."""
import math
import xml.etree.ElementTree as ET

CAR_FRONT=.21;CAR_REAR=.17;CAR_HALF_WIDTH=.215
WALL_MARGIN=.04  # Additional clearance outside the conservative body/wheel box.


def _pose(element):
    values=list(map(float,(element.findtext('pose') or '0 0 0 0 0 0').split()))
    if abs(values[3])+abs(values[4])>1e-4:raise ValueError('巡逻安全检查暂不支持倾斜地形碰撞盒')
    return values[0],values[1],values[2],values[5]


def _compose(parent,local):
    x,y,z,yaw=parent;u,v,w,turn=local;c,s=math.cos(yaw),math.sin(yaw)
    return x+c*u-s*v,y+s*u+c*v,z+w,yaw+turn


def terrain_boxes(world_path):
    world=ET.parse(world_path).getroot().find('world');boxes=[]
    saved={m.get('name'):m for m in world.findall('state/model')}
    def visit(model,parent,prefix):
        name=model.get('name','')
        if name.startswith(('Move','red_cube','blue_cube','competition_camera','zone_')):return
        # Gazebo saved state overrides top-level model poses on load.
        # Original v2 contains both, with different room X coordinates.
        state=saved.get(name) if not prefix else None
        pose=_pose(state) if state is not None else _compose(parent,_pose(model));prefix=prefix+'/'+name
        for link in model.findall('link'):
            lp=_compose(pose,_pose(link))
            for collision in link.findall('collision'):
                size=collision.findtext('geometry/box/size')
                if not size:continue
                sx,sy,sz=map(float,size.split());cp=_compose(lp,_pose(collision))
                # Include partitions and crates intersecting the moving car's
                # height; ground planes do not have box geometry.
                if cp[2]+sz/2<=.01 or cp[2]-sz/2>=.72:continue
                boxes.append(dict(name=prefix+'/'+link.get('name',''),pose=cp,size=(sx,sy,sz)))
        for child in model.findall('model'):visit(child,pose,prefix)
    for model in world.findall('model'):visit(model,(0.,0.,0.,0.),'')
    if not boxes:raise ValueError('没有读取到地形碰撞盒，禁止跳过巡逻轨迹检查')
    return boxes


def _point_segment(p,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/length)) if length else 0.
    return math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)


def segment_box_clearance(a,b,box):
    x,y,_,yaw=box['pose'];c,s=math.cos(yaw),math.sin(yaw)
    def local(p):return c*(p[0]-x)+s*(p[1]-y),-s*(p[0]-x)+c*(p[1]-y)
    a,b=local(a),local(b);hx,hy=[v/2 for v in box['size'][:2]]
    lo,hi=0.,1.;intersects=True
    for axis,half in enumerate((hx,hy)):
        delta=b[axis]-a[axis]
        if abs(delta)<1e-12:
            if abs(a[axis])>half:intersects=False;break
        else:
            u,v=sorted(((-half-a[axis])/delta,(half-a[axis])/delta))
            lo,hi=max(lo,u),min(hi,v)
            if lo>hi:intersects=False;break
    if intersects:return 0.
    corners=[(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]
    return min(min(_point_segment(a,p,q),_point_segment(b,p,q),
                   _point_segment(p,a,b),_point_segment(q,a,b))
               for p,q in zip(corners,corners[1:]+corners[:1]))


def swept_car_box(patrol):
    a,b=patrol['start'],patrol['end'];length=math.dist(a[:2],b[:2])
    yaw=math.atan2(b[1]-a[1],b[0]-a[0]);offset=(CAR_FRONT-CAR_REAR)/2
    return dict(pose=((a[0]+b[0])/2+offset*math.cos(yaw),
                      (a[1]+b[1])/2+offset*math.sin(yaw),0.,yaw),
                size=(length+CAR_FRONT+CAR_REAR,2*CAR_HALF_WIDTH,.72))


def _corners(box):
    x,y,_,yaw=box['pose'];hx,hy=[v/2 for v in box['size'][:2]];c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*u-s*v,y+s*u+c*v) for u,v in [(-hx,-hy),(hx,-hy),(hx,hy),(-hx,hy)]]


def boxes_clearance(first,second):
    a,b=_corners(first),_corners(second)
    return min([segment_box_clearance(p,q,second) for p,q in zip(a,a[1:]+a[:1])]
               +[segment_box_clearance(p,q,first) for p,q in zip(b,b[1:]+b[:1])])


def patrol_reports(patrols,world_path,require_safe=True):
    boxes=terrain_boxes(world_path);reports=[]
    for patrol in patrols:
        swept=swept_car_box(patrol)
        gap,wall=min((boxes_clearance(swept,box),box['name']) for box in boxes)
        centre=min(segment_box_clearance(patrol['start'],patrol['end'],box) for box in boxes)
        length=math.dist(patrol['start'],patrol['end'])
        safe=gap>=WALL_MARGIN and length>=2.
        record=dict(model=patrol['model'],start=list(patrol['start']),end=list(patrol['end']),length_m=length,
                    nearest_wall=wall,minimum_centre_wall_m=centre,
                    minimum_car_wall_margin_m=gap,required_margin_m=WALL_MARGIN,passed=safe)
        reports.append(record)
        if require_safe and not safe:raise ValueError(f"{patrol['model']} 巡逻轨迹穿墙或余量不足: {wall}, 车身余量 {gap:.3f}m")
    return reports
