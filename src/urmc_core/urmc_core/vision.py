"""Experimental HSV/geometry perception; thresholds need real camera calibration."""
import cv2
cv2.setNumThreads(1)
import numpy as np
from .barrier import detect as detect_barrier


def analyze(image, white_s=60, white_v=180, yellow_low=18, yellow_high=38):
    if image is None or image.ndim!=3 or image.shape[2]!=3 or min(image.shape[:2])<40:
        raise ValueError('Expected BGR image of at least 40x40 pixels')
    image=cv2.resize(image,(320,240));h,w=image.shape[:2]
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    yellow=cv2.inRange(hsv,(yellow_low,90,90),(yellow_high,255,255))
    white=cv2.inRange(hsv,(0,0,white_v),(179,white_s,255))
    roi=(slice(int(.76*h),h),slice(int(.05*w),int(.95*w)))
    def center(mask):
        ys,xs=np.nonzero(mask[roi]);return float(xs.mean()+int(.05*w)) if len(xs)>=25 else None
    yx,wx=center(yellow),center(white)
    valid=yx is not None and wx is not None and 35<wx-yx<240
    error=((yx+wx)/2-w/2)/(w/2) if valid else 0.
    # Alternating black/white tiles, inside lane: reject a plain white stripe.
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    checker_rows=0
    for row in gray[180:224:4,90:230]:
        bits=row>180;dark=row<65
        transitions=np.count_nonzero(bits[1:]!=bits[:-1])
        if transitions>=5 and .25<bits.mean()<.75 and dark.mean()>.25:checker_rows+=1
    red=cv2.inRange(hsv,(0,100,90),(10,255,255)) | cv2.inRange(hsv,(170,100,90),(179,255,255))
    contours,_=cv2.findContours(red[:180],cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    barrier=False;stop=False
    for c in contours:
        x,y,cw,ch=cv2.boundingRect(c);area=cv2.contourArea(c)
        if area<100:continue
        if cw>120 and cw/max(ch,1)>4:barrier=True
        # Shape candidate only: an octagonal red sign; text recognition not implemented.
        approx=cv2.approxPolyDP(c,.025*cv2.arcLength(c,True),True)
        if 7<=len(approx)<=9 and .7<cw/max(ch,1)<1.3 and area>250:stop=True
    intersection=bool(np.any(np.count_nonzero(white[130:175],axis=1)>240))
    blue=detect_barrier(image)
    barrier=blue['position']=='CLOSED'
    return dict(lane_valid=valid,error=error,yellow_x=yx,white_x=wx,
                lane_width=wx-yx if valid else None,checker=checker_rows>=3,stop=stop,
                barrier=barrier,barrier_clear=blue['position']=='OPEN',
                barrier_position=blue['position'],barrier_angle=blue.get('angle'),intersection=intersection)
