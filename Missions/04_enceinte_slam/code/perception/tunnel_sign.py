"""Tunnel pictogram candidate, only for explicit stationary camera preparation."""
import cv2
import numpy as np
from .road_signs import _image

def detect_tunnel(image, cfg=None):
    """Arch geometry in a square board, matching figure 6, not a learned classifier."""
    image = _image(image)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray[:190], 35, 100)
    best = dict(visible=False, kind='tunnel', reason='Aucun pictogramme d’arche reconnu')
    for contour in cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]:
        area = cv2.contourArea(contour)
        if area < 350: continue
        quad = cv2.approxPolyDP(contour, .025*cv2.arcLength(contour,True), True)
        if len(quad) != 4 or not cv2.isContourConvex(quad): continue
        x,y,w,h = cv2.boundingRect(quad)
        if min(w,h)<23 or not .70<w/h<1.4: continue
        p = quad.reshape(4,2).astype(np.float32)
        sums=p.sum(axis=1);diff=np.diff(p,axis=1).ravel()
        ordered=np.array([p[sums.argmin()],p[diff.argmin()],p[sums.argmax()],p[diff.argmax()]],np.float32)
        warp=cv2.getPerspectiveTransform(ordered,np.array([[0,0],[63,0],[63,63],[0,63]],np.float32))
        v=cv2.warpPerspective(hsv[:,:,2],warp,(64,64))/255.
        yy,xx=np.indices((64,64));xx=(xx+.5)/64;yy=(yy+.5)/64
        dx=xx-.5;dy=yy-.54;rr=np.hypot(dx,dy)
        ring=(yy<.55)&(rr>.225)&(rr<.40)
        pillars=((xx>.12)&(xx<.28)|(xx>.72)&(xx<.88))&(yy>.55)&(yy<.84)
        door=((np.abs(dx)<.19)&(yy>.53)&(yy<.87))|((rr<.19)&(yy<=.54))
        outer=(yy>.13)&(yy<.27)&(np.abs(dx)>.35)
        lit=np.r_[v[ring],v[pillars]]
        dark=v[door]
        contrast=float(np.median(lit)-np.median(dark))
        # Require the curved upper band AND both posts; a square frame is not an arch.
        if contrast<.16 or np.mean(v[ring]>.36)<.57 or np.mean(v[pillars]>.36)<.65 or np.mean(dark<.42)<.72:
            continue
        if outer.any() and np.mean(v[outer]>.60)>.8: continue
        score=min(1., contrast+.40)
        if score>best.get('confidence',0):
            best=dict(visible=True,kind='tunnel',bbox=[x,y,w,h],confidence=round(score,3),
                      reason='Arche et deux montants sur panneau carré ; candidat à valider sur terrain')
    return best
