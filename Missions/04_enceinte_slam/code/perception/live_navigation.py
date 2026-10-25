"""Live LiDAR layers and local odometry view. No scan matching, no hidden SLAM."""
import math
import cv2,numpy as np
from .enclosure_map import Grid


def ray_masks(grid,origin,rays):
    free=np.zeros(grid.data.shape,np.uint8);hit=np.zeros_like(free)
    start=grid.cell(*origin)
    for x,y,occupied in rays:
        cell=grid.cell(x,y);cv2.line(free,start,cell,1,1)
        if occupied and grid.inside(cell):hit[cell[1],cell[0]]=1
    free[hit>0]=0
    return free.astype(bool),hit.astype(bool)


class LiveGridLayer:
    """Hits block immediately; clearing a saved obstacle requires distinct scans."""
    def __init__(self):self.key=None;self.scan_id=None;self.free_count=None;self.hits=None;self.revision=0;self.cleared=0
    def observe(self,grid,origin,rays,scan_id):
        key=(grid.w,grid.h,grid.res,grid.origin)
        if self.key!=key:
            self.key=key;self.free_count=np.zeros(grid.data.shape,np.uint8);self.hits=np.zeros(grid.data.shape,bool);self.scan_id=None
        if self.scan_id==scan_id:return False
        self.scan_id=scan_id;free,hit=ray_masks(grid,origin,rays)
        self.free_count[free]=np.minimum(3,self.free_count[free]+1);self.free_count[hit]=0
        self.hits[free]=False;self.hits[hit]=True;self.revision+=1
        return True
    def apply(self,grid):
        if self.key!=(grid.w,grid.h,grid.res,grid.origin):return grid
        cells=grid.data.copy();clear=(cells>=50)&(self.free_count>=3);cells[clear]=0;cells[self.hits]=100;self.cleared=int(np.count_nonzero(clear))
        return Grid(cells,grid.res,grid.origin)


class LocalView:
    """Fixed metric view relative to startup odometry; never a SLAM pose graph."""
    def __init__(self):
        self.grid=Grid(np.full((180,180),-1,np.int16),.04,(-3.6,-3.6,0.));self.scan_id=None;self.revision=0;self.current_hits=np.zeros(self.grid.data.shape,bool)
    def observe(self,origin,rays,scan_id):
        if scan_id==self.scan_id:return False
        self.scan_id=scan_id;free,hit=ray_masks(self.grid,origin,rays)
        self.grid.data[free]=0;self.grid.data[hit]=100;self.current_hits=hit;self.revision+=1
        return True


class CameraWatch:
    """Optional test assistance: quality and scene changes AT REST, no depth claim."""
    def __init__(self):
        self.seen=None;self.stamp=None;self.previous=None;self.brightness=None;self.sharpness=None;self.change=0.;self.hold_until=0.
    def image(self,image,now,stamp,moving=False,min_light=15,change_threshold=.12):
        if self.stamp is not None and stamp<=self.stamp:return False
        gray=cv2.cvtColor(cv2.resize(image,(160,120)),cv2.COLOR_BGR2GRAY)
        self.brightness=round(float(gray.mean()),1);self.sharpness=round(float(cv2.Laplacian(gray,cv2.CV_64F).var()),1);self.change=0.
        if not moving and self.previous is not None and self.seen is not None and 0<=now-self.seen<.5:
            self.change=float(np.count_nonzero(cv2.absdiff(gray[40:],self.previous[40:])>25)/gray[40:].size)
            if self.change>=change_threshold:self.hold_until=now+.8
        self.previous=None if moving else gray;self.seen=now;self.stamp=stamp
        return True
    def report(self,now,min_light=15):
        age=now-self.seen if self.seen is not None else None;fresh=age is not None and 0<=age<.5
        allowed=fresh and self.brightness>=min_light and now>=self.hold_until
        state='ATTENTE_IMAGE' if not fresh else 'IMAGE_SOMBRE' if self.brightness<min_light else 'CHANGEMENT_A_L_ARRET' if now<self.hold_until else 'IMAGE_RECENTE'
        return dict(enabled=True,fresh=fresh,allowed=bool(allowed),state=state,age=round(age,3) if age is not None else None,brightness=self.brightness,sharpness=self.sharpness,change_ratio=round(self.change,3),pause_remaining=round(max(0.,self.hold_until-now),2))


def validate_destination(route,grid,radius):
    point=route.get('exit')
    if not isinstance(point,(list,tuple)) or len(point)!=2 or not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) for v in point):raise ValueError('Choisir le point d’arrivée sur la carte.')
    cell=grid.cell(*point)
    if not grid.inside(cell) or grid.blocked(radius)[cell[1],cell[0]]:raise ValueError('Arrivée inconnue, occupée ou trop proche d’un obstacle.')
    return [float(v) for v in point]
