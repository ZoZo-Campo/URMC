import math,pytest
from urmc_core.enclosure_layout import validate_layout,contains,outside,validate_enclosure_route
L=dict(width=1.,depth=1.25,origin=[0.,-.5,0.])
R=dict(entry=[-.18,0.],inside=[.7,0.],exit=[1.58,0.])

def test_outside_inside_outside_route_and_physical_margins():
 assert validate_enclosure_route(L,R,(-.4,0,0),.13)==L
 assert contains(L,(.7,0),.13) and outside(L,(1.58,0),.13)
 assert not outside(L,(-.1,0),.13)

@pytest.mark.parametrize('bad',[None,{},dict(width=True,depth=1.25,origin=[0,0,0]),dict(width=1,depth=float('nan'),origin=[0,0,0])])
def test_dimensions_are_not_assumed_or_nonfinite(bad):
 with pytest.raises(ValueError):validate_layout(bad)

@pytest.mark.parametrize('key,point',[('entry',[.2,0]),('inside',[-.3,0]),('exit',[1.1,0])])
def test_wrong_side_waypoint_rejected(key,point):
 with pytest.raises(ValueError):validate_enclosure_route(L,dict(R,**{key:point}),(-.4,0,0),.13)

def test_cannot_start_a_full_mission_inside():
 with pytest.raises(ValueError):validate_enclosure_route(L,R,(.4,0,0),.13)

def test_rotated_enclosure_uses_its_own_axes():
 e=dict(width=1.,depth=1.25,origin=[2.,3.,math.pi/2])
 assert contains(e,(1.5,3.7),.13) and outside(e,(2.,2.6),.13)
