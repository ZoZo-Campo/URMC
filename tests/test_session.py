import pytest
from urmc_core.session import Session,validate_changes

def test_waiting_at_start():
    s=Session();s.advance(.1);assert s.elapsed==0 and not s.running

def test_pause_freezes_time():
    s=Session();s.action('start');s.advance(.1);s.action('pause');s.advance(20);assert s.elapsed==.1

def test_restart_waits_and_increments_epoch():
    s=Session(True);s.advance(.1);s.action('restart');assert s.elapsed==0 and not s.running and s.epoch==1

def test_rearm_has_new_revision():
    s=Session(True);s.action('start');assert s.revision==1 and s.running

@pytest.mark.parametrize('value',[float('nan'),-1,601,True,'300'])
def test_invalid_latency(value):
    with pytest.raises(ValueError):validate_changes({'camera_latency_ms':value})

def test_invalid_control_key():
    with pytest.raises(ValueError):validate_changes({'cmd_vel':1})

def test_boolean_strict():
    with pytest.raises(ValueError):validate_changes({'camera_off':'false'})

def test_valid_changes():assert validate_changes({'camera_off':True,'brightness':.5})=={'camera_off':True,'brightness':.5}
