import cv2
import numpy as np
import pytest
from urmc_core.vision import analyze
from urmc_core.sim import scene

def test_lane_from_pixels():
    o=analyze(scene(0,0,0,1,0));assert o['lane_valid'] and abs(o['error'])<.05

def test_shifted_lane():assert analyze(scene(0,.05,0,1,0))['error']>0

def test_checker():assert analyze(scene(2.5,0,0,1,0))['checker']

def test_plain_stripe_not_checker():
    im=scene(0,0,0,1,0);cv2.rectangle(im,(90,180),(230,224),(240,240,240),-1)
    assert not analyze(im)['checker']

def test_barrier_pixels():assert analyze(scene(.4,0,0,2,5))['barrier']

def test_open_barrier_pixels():assert not analyze(scene(.4,0,0,2,10))['barrier']

def test_intersection_widens_then_has_no_lines():
    approach=analyze(scene(.2,0,0,3,5));entry=analyze(scene(.4,0,0,3,5))
    assert entry['lane_width']>1.4*approach['lane_width']
    assert not analyze(scene(.8,0,0,3,5))['lane_valid']

def test_stop_shape_candidate():assert analyze(scene(2.5,0,0,1,0,'stop'))['stop']

def test_no_marker_false_positive():
    o=analyze(scene(0,0,0,1,0));assert not any(o[k] for k in ['checker','stop','barrier','intersection'])

def test_missing_white():
    im=scene(0,0,0,1,0);im[:,180:]=(65,65,65)
    assert not analyze(im)['lane_valid']

def test_invalid_image():
    with pytest.raises(ValueError):analyze(None)
