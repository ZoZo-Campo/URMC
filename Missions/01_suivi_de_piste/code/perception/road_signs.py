"""Conservative pixel candidates and temporal confirmation; no motor or ROS code.

STOP octagons and the red/white no-entry panel pictured in the supplied rules
are supported. A red patch alone is never sufficient.
"""
import math
import cv2
import numpy as np


def _image(image):
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError('Image BGR attendue')
    return cv2.resize(image, (320, 240))


def detect_stop(image, cfg=None):
    cfg = cfg or {}
    image = _image(image)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 90, 55), (12, 255, 255)) | cv2.inRange(hsv, (168, 90, 55), (179, 255, 255))
    # Exclude the floor and the lower part of the supporting post.
    mask[190:] = 0
    best = dict(visible=False, kind='stop', reason='Aucun panneau rouge confirmé')
    for contour in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area < cfg.get('sign_area_min', 150) or min(w, h) < 15 or not .72 < w / h < 1.38:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, .025 * perimeter, True)
        solidity = area / max(cv2.contourArea(cv2.convexHull(contour)), 1)
        circularity = 4 * math.pi * area / max(perimeter * perimeter, 1)
        if not 7 <= len(approx) <= 12 or solidity < .83 or circularity < .65:
            continue
        inside = np.zeros(mask.shape, np.uint8)
        cv2.drawContours(inside, [contour], -1, 255, -1)
        inside = cv2.erode(inside, np.ones((3, 3), np.uint8))
        white = cv2.inRange(hsv, (0, 0, 100), (179, 140, 255)) & inside
        crop = white[y:y+h, x:x+w]
        center = crop[int(h*.20):max(int(h*.82), 1), int(w*.12):max(int(w*.88), 1)]
        ratio = np.count_nonzero(center) / max(center.size, 1)
        ys, xs = np.nonzero(center)
        if not cfg.get('sign_white_min', .035) <= ratio <= .62 or not len(xs):
            continue
        # Both lettering and a no-entry bar have a broad central white pattern.
        if (xs.max()-xs.min()) < w*.30:
            continue
        score = min(1., solidity*.45 + circularity*.35 + min(ratio/.15, 1)*.20)
        if score > best.get('confidence', 0):
            best = dict(visible=True, kind='stop', bbox=[x,y,w,h], area=round(area),
                        white_ratio=round(ratio,3), confidence=round(score,3),
                        reason='Forme rouge et motif blanc central')
    return best



class VisualConfirmation:
    def __init__(self):
        self.frames=0;self.since=None;self.last_time=None;self.last_sample=None;self.previous=None;self.confirmed=False
    def reset(self):
        self.frames=0;self.since=None;self.previous=None;self.confirmed=False
    def update(self,candidate,now,sample,cfg=None):
        cfg=cfg or {}
        if self.last_time is not None and not 0<=now-self.last_time<.35:self.reset()
        if self.last_sample is not None and sample<=self.last_sample:return self.confirmed
        self.last_sample=sample;self.last_time=now
        if not candidate.get('visible'):
            self.reset();return False
        box=candidate.get('bbox')
        coherent=True
        if self.previous and box:
            a,b=self.previous,box
            coherent=(math.hypot(a[0]+a[2]/2-b[0]-b[2]/2,a[1]+a[3]/2-b[1]-b[3]/2)<max(a[2],a[3])*.6
                      and .45<a[2]/max(b[2],1)<2.2)
        if not coherent:self.reset()
        self.previous=box;self.frames+=1
        if self.since is None:self.since=now
        self.confirmed=self.frames>=cfg.get('sign_confirm_frames',3) and now-self.since>=cfg.get('sign_confirm_time',.15)
        return self.confirmed


class FinishGate:
    def __init__(self,target='both'):
        if target not in ('both','checker','stop'):raise ValueError('Arrivée : both, checker ou stop')
        self.target=target;self.stop=VisualConfirmation();self.checker=VisualConfirmation();self.event=None;self.report={}
    def update(self,image,checker,now,sample,cfg=None):
        if self.event:return self.event
        candidate=detect_stop(image,cfg) if self.target!='checker' else dict(visible=False,kind='stop')
        eligible=candidate.get('visible') and candidate.get('area',0)>=(cfg or {}).get('sign_stop_area',600)
        candidate['ready_to_stop']=bool(eligible)
        stop_ok=self.stop.update(dict(candidate,visible=bool(eligible)),now,sample,cfg)
        checker_ok=self.checker.update(dict(visible=bool(checker)),now,sample,cfg)
        self.report=dict(target=self.target,stop=candidate,stop_frames=self.stop.frames,checker_frames=self.checker.frames)
        if self.target!='checker' and stop_ok:self.event=dict(kind='stop',reason='Panneau STOP confirmé ; arrêt final',**{k:v for k,v in candidate.items() if k not in ('kind','reason')})
        elif self.target!='stop' and checker_ok:self.event=dict(kind='checker',reason='Damier confirmé ; arrêt final')
        return self.event
