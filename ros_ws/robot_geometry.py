"""Convex navigation envelope of the original robot with its arm stowed.

URDF: body 0.32 x 0.32 m; control axle 0.14 m ahead of body centre;
wheel radius 0.065 m, outer half-width 0.210 m.  Add 0.01 m here;
Nav2's additional footprint padding is retained. Arm work is stationary.
"""
import math
NAV_FOOTPRINT=[(-.31,-.17),(-.075,-.22),(.075,-.22),(.16,-.06),
               (.16,.06),(.075,.22),(-.075,.22),(-.31,.17)]


def grasp_preflight_valid(cache,pose,target,world_yaw,now):
    """Reuse an IK preflight only while the same stopped geometry is current."""
    if not cache or pose is None or not 0<=now-cache['time']<2.:return False
    yaw_error=(pose[2]-cache['base'][2]+math.pi)%(2*math.pi)-math.pi
    return (math.dist(pose[:2],cache['base'][:2])<.006 and abs(yaw_error)<.01
            and math.dist(target,cache['target'])<.003
            and abs(world_yaw-cache['world_yaw'])<1e-6)


def front_corridor_distance(scan,half_width=.27):
    """Nearest axial lidar return intersecting the actual swept body corridor.

    Infinity is a valid clear ray; NaN/invalid measurements are not clearance.
    Distances retain the laser origin (.14 m behind the control axle).
    """
    distances=[]
    for i,value in enumerate(scan.ranges):
        bearing=(scan.angle_min+i*scan.angle_increment+math.pi)%(2*math.pi)-math.pi
        if abs(bearing)>math.pi/2:continue
        if math.isinf(value) and value>0:value=scan.range_max
        if not math.isfinite(value) or not scan.range_min<value<=scan.range_max:continue
        if abs(value*math.sin(bearing))<=half_width:
            distances.append(value*math.cos(bearing))
    return min(distances) if distances else None


def braking_speed(clearance,approaching_speed=0.,reaction=.25,deceleration=1.2,stand_off=.25):
    """Largest speed whose braking envelope leaves the requested bumper gap.

    Includes reaction travel and an approaching obstacle's travel while braking.
    Assumes the obstacle retains its measured closing speed during the stop.
    Nav2 still checks the curved footprint trajectory and moving obstacles.
    """
    u=max(0.,approaching_speed);a=deceleration
    available=clearance-stand_off-u*reaction
    if available<=0:return 0.
    b=a*reaction+u
    return max(0.,math.sqrt(b*b+2*a*available)-b)

def point_in_footprint(x, y):
    """Return whether a base_control-frame floor point lies under the chassis."""
    inside=False
    j=len(NAV_FOOTPRINT)-1
    for i,(xi,yi) in enumerate(NAV_FOOTPRINT):
        xj,yj=NAV_FOOTPRINT[j]
        if (yi>y)!=(yj>y) and x < (xj-xi)*(y-yi)/(yj-yi)+xi:
            inside=not inside
        j=i
    return inside


def lidar_front_lane_clear(scan, distance):
    """Laser x=0 rad is forward; include its offset behind the control axle."""
    half_angle=min(.60,math.atan2(.25,max(.30,distance))+.10)
    ranges=[]
    for i,value in enumerate(scan.ranges):
        bearing=(scan.angle_min+i*scan.angle_increment+math.pi)%(2*math.pi)-math.pi
        if abs(bearing)<=half_angle and math.isfinite(value) and scan.range_min<value<scan.range_max:
            ranges.append(float(value))
    return len(ranges)>=3 and min(ranges)>distance+.30


def reachable_drop_candidates(zone, preferred, pose, occupied):
    """Interior landing points reachable from the stopped base, with cargo gaps.

    Geometry only proposes points. MoveIt must still validate both descent poses.
    The floor cube half size is .025 m; all proposals retain >= .055 m to edges.
    """
    zx,zy=zone;c,s=math.cos(pose[2]),math.sin(pose[2])
    candidates=[tuple(preferred)]
    candidates += [(zx+ix*.04,zy+iy*.04) for ix in range(-9,10) for iy in range(-4,5)]
    valid=[]
    for point in candidates:
        x,y=point
        if abs(x-zx)>.40 or abs(y-zy)>.17:continue
        if any(math.dist(point,other)<.12 for other in occupied):continue
        dx,dy=x-pose[0],y-pose[1];local=(c*dx+s*dy,-s*dx+c*dy)
        if point_in_footprint(*local):continue
        clearance=footprint_clearance(local)
        radial=math.hypot(local[0]+.14,local[1])
        if clearance>=.06 and .16<radial<.40:
            if point not in valid:valid.append(point)
    return sorted(valid,key=lambda point:math.dist(point,preferred))


def footprint_clearance(point):
    clearance=float('inf')
    for a,b in zip(NAV_FOOTPRINT,NAV_FOOTPRINT[1:]+NAV_FOOTPRINT[:1]):
        vx,vy=b[0]-a[0],b[1]-a[1]
        t=max(0.,min(1.,((point[0]-a[0])*vx+(point[1]-a[1])*vy)/(vx*vx+vy*vy)))
        clearance=min(clearance,math.dist(point,(a[0]+t*vx,a[1]+t*vy)))
    return -clearance if point_in_footprint(*point) else clearance


def delivery_region_approaches(zone, occupied):
    """Joint landing/docking proposals; keep existing cargo clear of the base.

    Map reachability and route selection are applied by the caller. Occupied
    positions are previously delivered cargo only, not hidden floor objects.
    """
    zx,zy=zone;result=[]
    for dx in (0.,-.16,.16,-.32,.32):
        for dy in (0.,-.16,.16):
            landing=(zx+dx,zy+dy)
            if any(math.dist(landing,p)<.12 for p in occupied):continue
            for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
                c,s=math.cos(yaw),math.sin(yaw)
                base=(landing[0]-.24*c,landing[1]-.24*s,yaw)
                clear=True
                for point in occupied:
                    vx,vy=point[0]-base[0],point[1]-base[1]
                    if footprint_clearance((c*vx+s*vy,-s*vx+c*vy))<.10:
                        clear=False;break
                if clear:result.append((landing,base))
    return result


def departure_direction(pose, landing):
    """Leave away from the actual landing: reverse for front cargo, forward for rear."""
    forward=(landing[0]-pose[0])*math.cos(pose[2])+(landing[1]-pose[1])*math.sin(pose[2])
    return -1 if forward>=0 else 1


def square_gripper_yaw(base_yaw):
    """Nearest world cardinal jaw axis; square cargo does not require a base turn."""
    return round(base_yaw/(math.pi/2))*(math.pi/2)


def downward_gripper_quaternion(base_yaw,world_yaw):
    """World Rz(world_yaw)*Ry(pi), expressed in the planar base frame."""
    offset=(base_yaw-world_yaw+math.pi)%(2*math.pi)-math.pi
    return (math.sin(offset/2),math.cos(offset/2),0.,0.)


def usable_drop_candidates(grid,zone,preferred,pose,occupied):
    """Only stop where placement AND a 30 cm departure leave turning space."""
    if not grid.safe(pose):return []
    valid=[]
    for landing in reachable_drop_candidates(zone,preferred,pose,occupied):
        # Rear geometric reach can fold the arm through its own chassis. Only
        # end navigation early in the tested forward placement workspace.
        forward=(landing[0]-pose[0])*math.cos(pose[2])+(landing[1]-pose[1])*math.sin(pose[2])
        if forward<.10:continue
        direction=departure_direction(pose,landing)
        exit_pose=(pose[0]+direction*.30*math.cos(pose[2]),
                   pose[1]+direction*.30*math.sin(pose[2]),pose[2])
        if not grid.safe(exit_pose) or not grid.direct_base_path_clear(pose,exit_pose,min_clearance=.43):continue
        # The complete stowed footprint must be able to rotate after leaving.
        if any(math.dist(exit_pose[:2],point)<.46 for point in [*occupied,landing]):continue
        valid.append(landing)
    return valid
