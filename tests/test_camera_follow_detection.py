"""Regression checks for the standalone camera controller's real-lighting detector."""
import importlib.util
from pathlib import Path
import cv2
import numpy as np
import pytest

script=Path(__file__).resolve().parents[1]/'kit_robot/suivi_piste_camera.py'
spec=importlib.util.spec_from_file_location('camera_follow_detection',script)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def sample(gain=1.,yellow=True,white=True):
 image=np.full((240,320,3),65,np.uint8)
 if yellow:cv2.line(image,(80,100),(80,239),(0,200,220),8)
 if white:cv2.line(image,(240,100),(240,239),(175,175,175),8)
 image=np.clip(image.astype(float)*gain,0,255).astype(np.uint8)
 ok,jpeg=cv2.imencode('.jpg',image);assert ok
 return jpeg

@pytest.mark.parametrize('gain',[.65,.8,1.,1.2])
def test_white_tape_visible_in_changed_exposure(gain):
 result=module.lane_details(sample(gain))
 assert result['error'] is not None
 assert abs(result['error'])<.04

@pytest.mark.parametrize('yellow,white',[(False,True),(True,False)])
def test_requires_both_borders(yellow,white):
 assert module.lane(sample(yellow=yellow,white=white)) is None

def test_no_border_stops():
 assert module.lane(sample(yellow=False,white=False)) is None

def test_refuses_invalid_jpeg():
 assert module.lane(b'not an image') is None

@pytest.mark.parametrize('left_color', ['yellow','white'])
def test_color_does_not_define_side(left_color):
 image=np.full((240,320,3),65,np.uint8)
 colors={'yellow':(0,200,220),'white':(175,175,175)}
 cv2.line(image,(80,100),(80,239),colors[left_color],8)
 cv2.line(image,(240,100),(240,239),colors['white' if left_color=='yellow' else 'yellow'],8)
 tracker=module.LaneTracker()
 assert abs(module.lane_details(cv2.imencode('.jpg',image)[1],tracker)['error'])<.04

def test_single_centered_border_without_history_is_ambiguous():
 image=np.full((240,320,3),65,np.uint8)
 cv2.line(image,(160,100),(160,239),(0,200,220),8)
 assert module.lane_details(cv2.imencode('.jpg',image)[1])['error'] is None


def test_rejects_sudden_border_jump_then_reacquires(monkeypatch):
 clock=[100.];monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
 tracker=module.LaneTracker()
 assert module.lane_details(sample(),tracker)['error'] is not None
 image=np.full((240,320,3),65,np.uint8)
 cv2.line(image,(200,100),(200,239),(0,200,220),8)
 cv2.line(image,(310,100),(310,239),(175,175,175),8)
 jpeg=cv2.imencode('.jpg',image)[1]
 assert module.lane_details(jpeg,tracker)['error'] is None
 clock[0]+=.6
 assert module.lane_details(jpeg,tracker)['error'] is not None
