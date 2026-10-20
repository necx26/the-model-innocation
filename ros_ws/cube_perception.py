"""RGB floor-cube validation without simulator identities or object locations."""
import numpy as np
import cv2


def fit_floor_cube_box(box,intrinsics,origin,camera_rotation,initial_xy):
    """Fit an axis-aligned 5 cm cube to a close RGB bounding box on the floor.

    All eight projected corners constrain the centre. This removes the lateral
    bias of projecting one bottom pixel then correcting only its dominant axis.
    No simulator pose or object identity is used. Poor shape fits are rejected.
    """
    x,y,w,h=box
    if min(w,h)<12:return None
    measured=np.asarray([x,y,x+w-1,y+h-1],dtype=float)
    corners=np.asarray([(dx,dy,z) for dx in (-.025,.025) for dy in (-.025,.025) for z in (0.,.05)])
    k=intrinsics;origin=np.asarray(origin);rotation=np.asarray(camera_rotation)
    centre=np.asarray(initial_xy,dtype=float)
    best=None
    for radius,step in ((.06,.005),(.005,.001)):
        offsets=np.arange(-radius,radius+step*.1,step)
        dx,dy=np.meshgrid(offsets,offsets)
        candidates=centre+np.column_stack((dx.ravel(),dy.ravel()))
        positions=np.column_stack((candidates,np.zeros(len(candidates))))
        camera=(positions[:,None,:]+corners[None,:,:]-origin)@rotation
        forward=camera[:,:,0]
        u=k[2]-k[0]*camera[:,:,1]/np.maximum(.001,forward)
        v=k[5]-k[4]*camera[:,:,2]/np.maximum(.001,forward)
        edges=np.column_stack((u.min(axis=1),v.min(axis=1),u.max(axis=1),v.max(axis=1)))
        errors=np.mean((edges-measured)**2,axis=1)
        errors[np.any(forward<.05,axis=1)]=float('inf')
        index=int(np.argmin(errors));centre=candidates[index]
        best=(centre,float(np.sqrt(errors[index])))
    if best[1]>1.5 or np.linalg.norm(best[0]-np.asarray(initial_xy))>.065:return None
    return float(best[0][0]),float(best[0][1]),best[1]


def strict_color_candidates(image):
    """Require saturated target colors, compact shapes and solid color fill."""
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    masks={
        'red':cv2.inRange(hsv,(0,190,65),(5,255,255)) |
              cv2.inRange(hsv,(175,190,65),(179,255,255)),
        'blue':cv2.inRange(hsv,(110,190,45),(130,255,255)),
    }
    result=[]
    for color,mask in masks.items():
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour)<8:continue
            x,y,w,h=cv2.boundingRect(contour)
            # A clipped silhouette has no reliable dimensions. This also keeps
            # carried cargo / robot parts at the frame edge out of object memory.
            if x<=1 or y<=1 or x+w>=image.shape[1]-1 or y+h>=image.shape[0]-1:continue
            if not .5<w/max(1,h)<1.9:continue
            purity=float(np.count_nonzero(mask[y:y+h,x:x+w]))/(w*h)
            if purity<.60:continue
            result.append(dict(color=color,box=[x,y,w,h],color_purity=round(purity,3),
                               reason='等待坐标投影'))
    return result


def floor_cube_rejection(hsv, box, forward_distance, fx, fy):
    x, y, w, h = box
    if forward_distance <= 0:
        return '目标不在相机前方'
    # A 5 cm cube can expose two faces. Allow pixel quantization and oblique
    # views, but exclude wall-size patches and small texture specks.
    width = w * forward_distance / fx
    height = h * forward_distance / fy
    if not (.018 <= width <= .115 and .018 <= height <= .115):
        return '实际尺寸不符合5厘米方块'
    # The bottom of a floor object must meet the neutral ground. A red patch
    # within wood or brick still has textured wall immediately underneath it.
    gap = max(1, min(3, h//8))
    top = y+h+gap
    bottom = min(hsv.shape[0], top+max(3, min(8, h//3)))
    left = max(0, x+w//5)
    right = min(hsv.shape[1], x+w-w//5)
    support = hsv[top:bottom, left:right]
    if support.size == 0:
        return '目标下方没有可核验的地面'
    floor = (support[:, :, 1] < 50) & (support[:, :, 2] > 25)
    if float(np.mean(floor)) < .55:
        return '颜色位于墙面或非地面背景'
    return None


def confirmed_near_target(candidate, target, max_distance=.25):
    """Refine a multi-frame confirmed target from a close image despite floor occlusion."""
    if int(target.get('hits',0)) < 2 or candidate.get('color') != target.get('color'):
        return None
    if candidate.get('reason') != '颜色位于墙面或非地面背景':
        return None
    point=candidate.get('ground_projection')
    if not isinstance(point,(list,tuple)) or len(point)<2:
        return None
    x,y=map(float,point[:2])
    if not np.isfinite([x,y]).all() or np.hypot(x-target['x'],y-target['y'])>max_distance:
        return None
    return x,y
