import numpy as np
from urmc_core.entrance import find_opening,EntranceGate
from urmc_core.operator_settings import DEFAULTS

def walls(width=.5,blocked=False):
 points=[(0,float(y)) for y in np.r_[np.linspace(-.65,-width/2,40),np.linspace(width/2,.65,40)]]
 if blocked:points.extend((0,float(y)) for y in np.linspace(-.23,.23,30))
 return points

def test_two_supported_jambs_are_required():
 found=find_opening(walls(),(-.3,0),(1.,0),.13)
 assert found['visible'] and .49<found['width']<.52
 assert found['goal'][0]>.13 and abs(found['goal'][1])<.02
 assert not find_opening(walls()[:40],(-.3,0),(1.,0),.13)['visible']
 assert not find_opening(walls(blocked=True),(-.3,0),(1.,0),.13)['visible']
 assert not find_opening(walls(.20),(-.3,0),(1.,0),.13)['visible']

def test_rotated_opening_and_both_robot_margins():
 points=[(2-y,3+x) for x,y in walls()]
 found=find_opening(points,(2.,2.7),(2.,4.),.13)
 assert found['visible'] and found['normal'][1]>.9
 assert not find_opening(points,(2.,2.7),(2.,4.),.3)['visible']

def test_repeated_scans_and_stale_gap_cannot_confirm_a_portal():
 gate=EntranceGate()
 def update(t,stamp,points=None):return gate.update(points or walls(),(-.3,0),(1.,0),.13,t,stamp,DEFAULTS)
 assert not update(0,1)['confirmed']
 assert not update(.1,1)['confirmed']
 assert not update(.2,1)['confirmed']
 assert not update(.5,2)['confirmed']
 assert not update(.6,3)['confirmed']
 assert update(.7,4)['confirmed']
 assert not update(.8,5,walls(blocked=True))['confirmed']


def test_opening_from_real_bench_rays_with_an_off_axis_inside_point():
 import sys,math
 from pathlib import Path
 sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
 from sim_enclosure_slam import WALLS
 from urmc_core.sim import ray_segment
 x,y=-.24,0.;points=[]
 for i in range(720):
  a=-math.pi+i*math.pi/360;r=min(ray_segment(x,y,a,s) for s in WALLS)
  if .05<r<3.48:points.append((x+r*math.cos(a),y+r*math.sin(a)))
 found=find_opening(points,(x,y),(.98,.30),.13)
 assert found['visible'] and abs(found['center'][0])<.04 and abs(found['center'][1])<.05
 assert .48<found['width']<.56
