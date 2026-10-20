"""Geometry and timing gates for stationary, region-based manipulation."""
import math



def floor_chassis_clearance(point):
    """Distance to original body/wheels at floor level in base_control frame.

    Navigation includes the raised, stowed arm; that larger polygon must not
    hide floor cubes that are actually clear of the chassis.
    """
    x,y=point;distances=[]
    for xmin,xmax,ymin,ymax in ((-.307,.02,-.16,.16),(-.065,.065,-.21,-.16),(-.065,.065,.16,.21)):
        dx=max(xmin-x,0.,x-xmax);dy=max(ymin-y,0.,y-ymax)
        distances.append(math.hypot(dx,dy))
    return min(distances)


def capture_geometry(pose,target,max_radial=.42):
    """A floor cube outside the base and within the arm's planar workspace.

    This proposes a pose only. Synchronized vision, static-map clearance,
    collision-aware IK and motion planning are mandatory before execution.
    """
    if pose is None:return False
    dx,dy=target[0]-pose[0],target[1]-pose[1]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    x,y=c*dx+s*dy,-s*dx+c*dy
    return floor_chassis_clearance((x,y))>=.04 and .18<math.hypot(x+.14,y)<max_radial


def forward_capture_stops(pose,target,max_distance=1.2):
    """First safe reachable poses along the current heading, without a base turn."""
    dx,dy=target[0]-pose[0],target[1]-pose[1]
    forward=dx*math.cos(pose[2])+dy*math.sin(pose[2])
    if forward<0 or math.dist(pose[:2],target)>max_distance:return []
    # Try close-to-centre workspace solutions as well as the earliest boundary.
    poses=[]
    for i in range(int(min(max_distance,forward+.15)/.01)+1):
        distance=i*.01
        p=(pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2]),pose[2])
        if capture_geometry(p,target,max_radial=.385):poses.append((distance,p))
    return poses


def gripper_feedback_ready(position,previous,target,elapsed,fresh):
    """Accept a reached jaw position, or a moving jaw stopped by contact.

    A stalled jaw never proves a grasp; contact distance and object lift are
    still checked by the controller. Stale samples cannot satisfy this gate.
    """
    if not fresh or elapsed<.20 or not math.isfinite(position):return False
    if abs(position-target)<.0006:return True
    return (target>0 and elapsed>=.28 and position>=.003 and previous is not None
            and abs(position-previous)<.00015)
