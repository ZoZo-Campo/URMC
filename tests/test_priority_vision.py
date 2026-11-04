"""Only mission 3 may use distant lines; handoff requires close lines."""
import sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from priority_vision import PriorityVision
from suivi_piste_camera import lane_details
from urmc_core.priority import PriorityBrain
from urmc_core.operator_settings import DEFAULTS

def image(near=False,far=False):
 im=np.full((240,320,3),60,np.uint8)
 for active,ys in [(near,(165,239)),(far,(95,145))]:
  if active:
   cv2.line(im,(80,ys[0]),(80,ys[1]),(0,200,220),6);cv2.line(im,(240,ys[0]),(240,ys[1]),(200,200,200),6)
 return cv2.imencode('.jpg',im)[1]

def test_far_lines_do_not_count_as_near_lines():
 vision=PriorityVision()
 for t in (.1,.2,.3):r=vision.observe(image(far=True),t)
 assert r['far_confirmed'] and r['far']['error'] is not None and r['near']['error'] is None
 assert lane_details(image(far=True))['error'] is not None # Legacy fallback stays unchanged.

def test_close_detection_takes_precedence_when_both_return():
 vision=PriorityVision()
 for t in (.1,.2,.3):r=vision.observe(image(near=True,far=True),t)
 assert r['far_confirmed'] and r['near']['error'] is not None
 assert r['near']['roi']==[156,230]

def test_frozen_or_interrupted_images_cannot_confirm_far():
 vision=PriorityVision();jpeg=image(far=True)
 for _ in range(4):r=vision.observe(jpeg,1.)
 assert not r['far_confirmed']
 vision.observe(jpeg,1.1);assert not vision.observe(jpeg,2.)['far_confirmed']

def crossing():
 b=PriorityBrain();b.state='CROSS';b.origin=(0.,0.);b.heading=0.;b.clear_frames=3;b.clear_since=0.;b.clear_scan=-1
 return b

def obs(t,**kw):
 return dict(fresh=True,pose=(.4,0,0),front=3.5,right=3.5,route_confirmed=True,sample_id=t,scan_id=t,lane_valid=False,error=None,**kw)

def test_far_guidance_is_bounded_and_never_finishes_crossing():
 b=crossing()
 for t in (1.,1.1,1.2,1.3):
  c=b.step(obs(t,far_confirmed=True,far_error=.4),t)
  assert c.v>0 and abs(c.w)<=.1 and b.state=='CROSS' and b.vision_source=='far'

def test_right_obstacle_overrides_far_guidance_and_requires_clearance():
 b=crossing();o=obs(1.,far_confirmed=True,far_error=.2);o['right']=.4
 assert b.step(o,1.).v==0
 assert b.step(obs(1.1,far_confirmed=True,far_error=.2),1.1).v==0
 assert b.clear_frames==1

def test_handoff_uses_three_close_images_and_ignores_far_steering():
 b=crossing()
 for t in (1.,1.1,1.2):
  o=obs(t,far_confirmed=True,far_error=.5);o.update(lane_valid=True,error=0.)
  b.step(o,t)
 assert b.state=='FOLLOW_AFTER_INTERSECTION' and b.vision_source=='near'
 o=obs(1.3,far_confirmed=True,far_error=.5);o.update(lane_valid=True,error=0.)
 c=b.step(o,1.3);assert c.v>0 and c.w==0


def test_one_decoded_image_is_shared_between_near_and_far(monkeypatch):
 jpeg=image(near=True,far=True);decoded=cv2.imdecode(jpeg,cv2.IMREAD_COLOR)
 monkeypatch.setattr(cv2,'imdecode',lambda *args:(_ for _ in ()).throw(AssertionError('decoded twice')))
 result=PriorityVision().observe(jpeg,.1,image=decoded)
 assert result['near']['error'] is not None and result['far']['error'] is not None
