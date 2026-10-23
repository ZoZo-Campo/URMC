"""Blue rectangular barrier orientation from central second-order moments.

Image axes only: no depth or clearance is inferred from an ellipse.
"""
import math
import cv2
import numpy as np


def blue_mask(image):
    """HSV bounds documented in loiccu/turtlebot3_line_following/track_detect.py."""
    hsv=cv2.cvtColor(cv2.resize(image,(320,240)),cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,(100,100,50),(130,255,255))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    return cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))


def detect(image):
    if image is None or image.ndim!=3:return dict(position='UNKNOWN',reason='image invalide')
    image=cv2.resize(image,(320,240))
    mask=blue_mask(image)
    pixels=int(cv2.countNonZero(mask));visible=pixels>=80
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    candidates=[]
    for contour in contours:
        area=cv2.contourArea(contour)
        if area<100:continue
        (_, _),(rw,rh),_=cv2.minAreaRect(contour)
        if min(rw,rh)<3 or max(rw,rh)<35 or area/(rw*rh)<.68:continue
        m=cv2.moments(contour)
        if m['m00']<=0:continue
        covariance=np.array([[m['mu20'],m['mu11']],[m['mu11'],m['mu02']]])/m['m00']
        values,vectors=np.linalg.eigh(covariance)
        if values[0]<=.1:continue
        ratio=float(values[1]/values[0])
        if ratio<1.2**2:continue
        axis=vectors[:,1];angle=math.degrees(math.atan2(axis[1],axis[0]))%180
        tilt=min(angle,180-angle)
        x,y,w,h=cv2.boundingRect(contour)
        position='CLOSED' if tilt<=25 else 'OPEN' if tilt>=65 else 'MOVING'
        candidates.append(dict(position=position,reason='rectangle bleu',visible=True,blue_pixels=pixels,angle=round(tilt,2),axis_angle=float(angle),
            elongation=round(math.sqrt(ratio),2),center=[m['m10']/m['m00'],m['m01']/m['m00']],
            ellipse_axes=[4*math.sqrt(float(values[1])),4*math.sqrt(float(values[0]))],
            bbox=[x,y,w,h],area=area))
    if not candidates:return dict(position='UNKNOWN',visible=visible,blue_pixels=pixels,reason='forme bleue non orientable' if visible else 'barriere hors champ')
    candidates.sort(key=lambda c:c['area'],reverse=True)
    if len(candidates)>1 and candidates[1]['area']>.55*candidates[0]['area'] and candidates[1]['position']!=candidates[0]['position']:
        return dict(position='UNKNOWN',visible=True,blue_pixels=pixels,reason='plusieurs rectangles bleus ambigus')
    return candidates[0]


class OpeningGate:
    """Closed first; stable raised state, or observed opening followed by absence."""
    def __init__(self,delay=1.,frames=5,allow_disappearance=False):
        self.closed_seen=False;self.opening_seen=False;self.since=None;self.frames=0
        self.last_sample=None;self.mode=None;self.delay=delay;self.minimum_frames=frames
        self.last_now=None;self.allow_disappearance=allow_disappearance
    def update(self,result,now,sample):
        position=result.get('position','UNKNOWN')
        if not math.isfinite(now):return False
        if self.last_now is not None and (now<self.last_now or now-self.last_now>.3):self.since=None;self.frames=0
        self.last_now=now
        if position=='CLOSED':self.closed_seen=True;self.opening_seen=False
        if self.closed_seen and position in ('OPEN','MOVING'):self.opening_seen=True
        absent=(self.allow_disappearance and self.opening_seen and result.get('visible') is False)
        mode='OPEN' if position=='OPEN' else 'ABSENT' if absent else None
        if not self.closed_seen or mode is None:
            self.since=None;self.frames=0;self.last_sample=sample;self.mode=None;return False
        if self.mode!=mode:self.since=None;self.frames=0
        self.mode=mode
        if self.since is None:self.since=now
        if sample!=self.last_sample:self.frames+=1;self.last_sample=sample
        return now-self.since>=self.delay and self.frames>=self.minimum_frames


def annotate(image,result,ready=False):
    out=cv2.resize(image,(320,240)).copy()
    position=result.get('position','UNKNOWN');color={'CLOSED':(40,40,240),'OPEN':(60,220,60),'MOVING':(0,180,240)}.get(position,(0,180,240))
    if 'bbox' in result:
        x,y,w,h=result['bbox'];cv2.rectangle(out,(x,y),(x+w,y+h),color,1)
        center=tuple(int(round(v)) for v in result['center'])
        axes=tuple(max(1,int(v/2)) for v in result['ellipse_axes'])
        cv2.ellipse(out,center,axes,result['axis_angle'],0,360,color,1)
        a=math.radians(result['axis_angle']);length=result['ellipse_axes'][0]/2
        tip=(int(center[0]+length*math.cos(a)),int(center[1]+length*math.sin(a)))
        cv2.line(out,center,tip,color,2)
    label={'CLOSED':'FERMEE','OPEN':'LEVEE','MOVING':'EN MOUVEMENT','UNKNOWN':'INCONNUE'}[position]
    if position=='UNKNOWN' and result.get('visible') is False:label='HORS CHAMP'
    cv2.rectangle(out,(0,0),(319,42),(25,30,30),-1)
    cv2.putText(out,label,(8,17),cv2.FONT_HERSHEY_SIMPLEX,.48,color,1)
    detail=f"angle {result.get('angle','?')} deg - ratio {result.get('elongation','?')}"
    cv2.putText(out,detail,(8,34),cv2.FONT_HERSHEY_SIMPLEX,.34,(230,230,230),1)
    if ready:cv2.putText(out,'OUVERTURE CONFIRMEE',(8,232),cv2.FONT_HERSHEY_SIMPLEX,.4,(60,220,60),1)
    mask=cv2.cvtColor(blue_mask(image),cv2.COLOR_GRAY2BGR)
    cv2.rectangle(mask,(0,0),(319,22),(25,30,30),-1)
    cv2.putText(mask,'MASQUE BLEU - pixels selectionnes',(6,15),cv2.FONT_HERSHEY_SIMPLEX,.36,(240,240,240),1)
    return np.hstack((out,mask))
