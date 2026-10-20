"""Persistent image-evidenced search coverage; no Gazebo object coordinates."""
import math
import cv2
import numpy as np


class SearchMemory:
    def __init__(self, points, confirmations=3):
        self.points=np.asarray(points,dtype=float).reshape(-1,2)
        self.confirmations=confirmations
        self.hits=np.zeros(len(self.points),dtype=np.uint8)
        self.seen=np.zeros(len(self.points),dtype=bool)
        self.empty=np.zeros(len(self.points),dtype=bool)
        self.present=np.zeros(len(self.points),dtype=bool)
        self.cleared=np.zeros(len(self.points),dtype=bool)
        self.last_frame=None

    def observe(self, indices, detections, frame):
        if frame==self.last_frame:return
        self.last_frame=frame
        indices=np.asarray(indices,dtype=int)
        near=np.zeros(len(self.points),dtype=bool)
        for d in detections:
            near |= np.linalg.norm(self.points-np.array([d['x'],d['y']]),axis=1)<.55
        # A newly visible object overrides an older empty/cleared observation.
        self.present[near]=True;self.empty[near]=False;self.cleared[near]=False
        self.hits[indices]=np.minimum(self.hits[indices].astype(int)+1,self.confirmations)
        confirmed=self.hits>=self.confirmations
        self.seen[confirmed]=True
        self.empty[confirmed & ~self.present & ~self.cleared]=True

    def collected(self, point, radius=.45):
        near=np.linalg.norm(self.points-np.array(point[:2]),axis=1)<=radius
        self.cleared[near]=True;self.empty[near]=False;self.present[near]=False
        self.seen[near]=True;self.hits[near]=self.confirmations

    def summary(self):
        return dict(unseen=int(np.count_nonzero(~self.seen)),
                    empty=int(np.count_nonzero(self.empty)),
                    occupied=int(np.count_nonzero(self.present & self.seen)),
                    collected=int(np.count_nonzero(self.cleared)),
                    checked=int(np.count_nonzero(self.seen)),total=len(self.points))


def observable_floor_points(points, origin, rot, k, image, line_free, detections=()):
    """Require full grid-cell FOV, adequate cube pixels, floor and clear sight.

    Coverage is sampled at 0.6 m grid cells. It is evidence for search planning,
    not a claim that an entire room or an occluded surface is empty.
    """
    points=np.asarray(points,dtype=float)
    if not len(points):return np.array([],dtype=int)
    origin=np.asarray(origin)
    xyz=np.column_stack((points,np.zeros(len(points))))
    camera=(xyz-origin)@rot
    distance=np.linalg.norm(points-origin[:2],axis=1)
    mask=(camera[:,0]>.25)&(distance>=.65)&(distance<=4.5)
    # The whole sampled cell and cube top must fit, not only its centre.
    for dx,dy,z in [(-.30,-.30,0),(-.30,.30,0),(.30,-.30,0),(.30,.30,0),(0,0,.05)]:
        sample=xyz+np.array([dx,dy,z])
        q=(sample-origin)@rot
        depth=np.maximum(q[:,0],1e-6)
        u=k[2]-k[0]*q[:,1]/depth;v=k[5]-k[4]*q[:,2]/depth
        mask &= (q[:,0]>.15)&(u>=5)&(u<image.shape[1]-5)&(v>=5)&(v<image.shape[0]-5)
    mask &= .05*k[0]/np.maximum(camera[:,0],1e-6)>=5
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    valid=[]
    for i in np.flatnonzero(mask):
        p=points[i]
        if not line_free(origin[:2],p):continue
        # A cell edge hidden behind a partition remains unknown.
        if not all(line_free(origin[:2],p+offset) for offset in
                   [(-.30,-.30),(-.30,.30),(.30,-.30),(.30,.30)]):continue
        q=camera[i];u=int(round(k[2]-k[0]*q[1]/q[0]));v=int(round(k[5]-k[4]*q[2]/q[0]))
        patch=hsv[v-3:v+4,u-3:u+4]
        floor=patch.size and float(np.mean((patch[:,:,1]<55)&(patch[:,:,2]>25)))>=.70
        positive=any(math.hypot(d['x']-p[0],d['y']-p[1])<.55 for d in detections)
        if floor or positive:valid.append(i)
    return np.asarray(valid,dtype=int)


def scan_clear_points(points, origin, yaw, ranges, angle_min, angle_increment, range_min, range_max):
    """Reject negative evidence hidden behind a live lidar obstacle."""
    points=np.asarray(points,dtype=float).reshape(-1,2)
    delta=points-np.asarray(origin[:2]);dist=np.linalg.norm(delta,axis=1)
    bearings=(np.arctan2(delta[:,1],delta[:,0])-yaw+math.pi)%(2*math.pi)-math.pi
    indices=np.rint((bearings-angle_min)/angle_increment).astype(int)
    values=np.asarray(ranges,dtype=float)
    valid=(indices>=0)&(indices<len(values))&(dist<range_max-.1)
    for i in np.flatnonzero(valid):
        # Include neighbors so a thin moving obstacle is not lost to rounding.
        local=values[max(0,indices[i]-1):min(len(values),indices[i]+2)]
        if np.isnan(local).any() or np.any(local<=range_min):valid[i]=False;continue
        if np.min(local)<dist[i]-.20:valid[i]=False
    return valid
