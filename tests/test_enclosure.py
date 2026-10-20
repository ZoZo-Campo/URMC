import pytest
from tests.evaluate_lidar import run

@pytest.mark.parametrize('offset,obstacle',[(0.,False),(.04,False),(-.04,False),(0.,True)])
def test_enclosure_variants(offset,obstacle):
    result=run(offset,obstacle)
    assert result['state']=='DONE'
    assert not result['collision']
    assert result['t']<60
