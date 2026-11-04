"""Find a measured gap with TWO supported jambs. Dimensions are never assumed.

World points have already been transformed with the measured laser extrinsics.
A/B provide a search direction, not a declaration that a door exists. No camera.
"""
import math
import numpy as np


def find_opening(points,approach,inside,radius,max_width=1.2):
    unknown=dict(visible=False,reason='Deux montants et une ouverture libre restent à détecter')
    if not points or approach is None or inside is None:return unknown
    a=np.asarray(approach[:2],float);b=np.asarray(inside[:2],float)
    direction=b-a;length=float(np.linalg.norm(direction))
    if length<.15:return unknown
    direction/=length
    data=np.asarray(points,float).reshape(-1,2)
    data=data[np.isfinite(data).all(axis=1)]
    relative=data-a;along=relative@direction
    data=data[(along>.04)&(along<.95)&(np.linalg.norm(relative,axis=1)<1.4)]
    if len(data)<8:return unknown
    angle=np.arctan2(data[:,1]-a[1],data[:,0]-a[0]);data=data[np.argsort(angle)]
    # Only depth discontinuities and ends of continuous surfaces can be jambs.
    discontinuities=np.linalg.norm(np.diff(data,axis=0),axis=1)>.075
    indices=np.unique(np.r_[0,np.where(discontinuities)[0],np.where(discontinuities)[0]+1,len(data)-1])
    ends=data[indices]
    if len(ends)>64:ends=ends[::math.ceil(len(ends)/64)]
    best=None
    for i,p in enumerate(ends):
        for q in ends[i+1:]:
            width=float(np.linalg.norm(q-p))
            if not 2*radius+.035<=width<=max_width:continue
            tangent=(q-p)/width;normal=np.array([tangent[1],-tangent[0]])
            if normal@direction<0:normal=-normal
            if normal@direction<.72:continue
            center=(p+q)/2;forward=float((center-a)@direction)
            sideways=abs(float(np.cross(direction,center-a)))
            if not .04<forward<.85 or sideways>max(.28,width*.55):continue
            u=(data-center)@normal;v=(data-center)@tangent
            on_wall=np.abs(u)<.03
            left=on_wall&(v<-width/2-.015)&(v>-width/2-.25)
            right=on_wall&(v>width/2+.015)&(v<width/2+.25)
            if np.count_nonzero(left)<2 or np.count_nonzero(right)<2:continue
            # A return in the proposed gap makes it an obstacle, not free space.
            if np.any((np.abs(u)<.065)&(np.abs(v)<width/2-.035)):continue
            # The proposed continuation must lie on the other side of the wall.
            if (b-center)@normal<radius:continue
            score=forward+sideways*.7
            if best is None or score<best[0]:
                goal=center+normal*(radius+.065)
                best=(score,dict(visible=True,center=center.tolist(),normal=normal.tolist(),jambs=[p.tolist(),q.tolist()],width=round(width,3),
                                 goal=goal.tolist(),reason='Ouverture mesurée entre deux montants ; passage libre'))
    return best[1] if best else unknown


class EntranceGate:
    def __init__(self):
        self.frames=0;self.last_sample=None;self.last_time=None;self.previous=None;self.result=dict(visible=False,confirmed=False,frames=0,reason='Approcher le repère A pour vérifier l’entrée')
    def update(self,points,approach,inside,radius,now,sample,cfg):
        if self.last_sample is not None and sample<=self.last_sample:return self.result
        if self.last_time is not None and not 0<=now-self.last_time<.35:self.frames=0;self.previous=None
        self.last_sample=sample;self.last_time=now
        found=find_opening(points,approach,inside,radius,cfg['m4_entry_width_max'])
        coherent=self.previous and found.get('visible') and math.dist(self.previous['center'],found['center'])<.065 and abs(self.previous['width']-found['width'])<.08
        self.frames=self.frames+1 if coherent else 1 if found.get('visible') else 0
        self.previous=found if found.get('visible') else None
        self.result=dict(found,frames=self.frames,confirmed=self.frames>=cfg['m4_entry_confirm_frames'])
        return self.result
