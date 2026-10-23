import cv2
import numpy as np
import pytest
from urmc_core.barrier import detect,OpeningGate,annotate


def rectangle(angle,color=(240,70,20),size=(130,18)):
    image=np.full((240,320,3),65,np.uint8)
    box=np.int32(cv2.boxPoints(((160,120),size,angle)))
    cv2.fillConvexPoly(image,box,color)
    return image

@pytest.mark.parametrize('angle,position',[(0,'CLOSED'),(15,'CLOSED'),(45,'MOVING'),(75,'OPEN'),(90,'OPEN'),(-15,'CLOSED'),(-75,'OPEN')])
def test_second_moment_angle(angle,position):
    result=detect(rectangle(angle))
    assert result['position']==position
    assert result['elongation']>3
    assert annotate(rectangle(angle),result).shape==(240,640,3)

@pytest.mark.parametrize('color',[(0,0,240),(230,230,230),(0,200,220),(65,65,65)])
def test_rejects_other_colors(color):assert detect(rectangle(0,color))['position']=='UNKNOWN'

@pytest.mark.parametrize('angle,position',[(0,'CLOSED'),(90,'OPEN')])
def test_wide_blue_panel(angle,position):
    assert detect(rectangle(angle,size=(120,75)))['position']==position

def test_rejects_blue_square():assert detect(rectangle(0,size=(50,50)))['position']=='UNKNOWN'

def test_rejects_conflicting_blue_rectangles():
    image=np.full((240,320,3),65,np.uint8)
    cv2.rectangle(image,(15,40),(145,58),(240,70,20),-1)
    cv2.rectangle(image,(250,60),(268,190),(240,70,20),-1)
    assert detect(image)['position']=='UNKNOWN'

def test_opening_requires_observed_closure():
    gate=OpeningGate()
    for i in range(25):assert not gate.update(dict(position='OPEN'),i*.1,i)

def test_stable_positive_opening_and_immediate_reclose():
    gate=OpeningGate();assert not gate.update(dict(position='CLOSED'),0,0)
    for i in range(1,11):assert not gate.update(dict(position='OPEN'),i*.1,i)
    assert gate.update(dict(position='OPEN'),1.2,12)
    assert not gate.update(dict(position='CLOSED'),1.3,13)

def test_absence_never_means_open():
    gate=OpeningGate();gate.update(dict(position='CLOSED'),0,0)
    for i in range(1,25):assert not gate.update(dict(position='UNKNOWN'),i*.1,i)

def test_repeated_frame_cannot_confirm_open():
    gate=OpeningGate();gate.update(dict(position='CLOSED'),0,0)
    for i in range(1,25):assert not gate.update(dict(position='OPEN'),i*.1,1)

def test_missing_frames_reset_open_confirmation():
    gate=OpeningGate(delay=.5);gate.update(dict(position='CLOSED'),0,0)
    for i in range(1,5):assert not gate.update(dict(position='OPEN'),i*.1,i)
    assert not gate.update(dict(position='OPEN'),2.,20)


def test_absence_after_observed_opening_needs_one_second():
    gate=OpeningGate(allow_disappearance=True)
    gate.update(dict(position='CLOSED',visible=True),0,0)
    gate.update(dict(position='MOVING',visible=True),.1,1)
    for i in range(2,12):assert not gate.update(dict(position='UNKNOWN',visible=False),i*.1,i)
    assert gate.update(dict(position='UNKNOWN',visible=False),1.3,13)


def test_unknown_visible_blue_never_means_clear():
    gate=OpeningGate(allow_disappearance=True)
    gate.update(dict(position='CLOSED',visible=True),0,0)
    gate.update(dict(position='MOVING',visible=True),.1,1)
    for i in range(2,30):assert not gate.update(dict(position='UNKNOWN',visible=True),i*.1,i)


def test_absence_without_any_opening_does_not_restart():
    gate=OpeningGate(allow_disappearance=True)
    gate.update(dict(position='CLOSED',visible=True),0,0)
    for i in range(1,30):assert not gate.update(dict(position='UNKNOWN',visible=False),i*.1,i)
