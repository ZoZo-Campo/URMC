import math,pytest
from urmc_core.enclosure_layout import validate_layout,contains,outside
L=dict(width=1.,depth=1.25,origin=[0.,-.5,0.])

def test_reference_location_uses_a_display_footprint():
 assert validate_layout(L)==L
 assert contains(L,(.7,0),.13) and outside(L,(1.58,0),.13)
 assert not outside(L,(-.1,0),.13)

@pytest.mark.parametrize('bad',[None,{},dict(width=True,depth=1.25,origin=[0,0,0]),dict(width=1,depth=float('nan'),origin=[0,0,0])])
def test_dimensions_are_not_assumed_or_nonfinite(bad):
 with pytest.raises(ValueError):validate_layout(bad)

def test_rotated_enclosure_uses_its_own_axes():
 e=dict(width=1.,depth=1.25,origin=[2.,3.,math.pi/2])
 assert contains(e,(1.5,3.7),.13) and outside(e,(2.,2.6),.13)
