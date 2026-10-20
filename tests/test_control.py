import math
from urmc_core.control import Brain, Command, guard, scan_features, sector

CLEAR=dict(front=3.5,left=3.5,right=3.5,rear=3.5,fl=3.5,fr=3.5)
def obs(**kw):return dict(fresh=True,lane_valid=True,error=0,**kw)

def test_lane_turn_direction():
    b=Brain(1);assert b.step(dict(fresh=True,lane_valid=True,error=.4),0).w<0

def test_missing_lane_stops():assert Brain(1).step({'fresh':True},0).v==0

def test_finish_requires_distinct_images():
    b=Brain(1)
    for t in [0,.1,.2]:b.step(obs(checker=True,sample_id=1),t)
    assert b.state=='FOLLOW'
    b.step(obs(checker=True,sample_id=2),.3);b.step(obs(checker=True,sample_id=3),.4)
    assert b.state=='DONE' and b.step(obs(),.5).v==0

def test_wrong_finish_does_not_stop():
    b=Brain(1)
    for t in [0,.1,.2]:b.step(obs(stop=True),t)
    assert b.state=='FOLLOW'

def test_barrier_requires_positive_open():
    b=Brain(2);b.step(obs(barrier=True),0)
    assert b.step(obs(barrier_clear=False,front=3.5),.1).v==0
    assert b.state=='WAIT_BARRIER'

def test_open_gap_restarts_confirmation():
    b=Brain(2);b.step(obs(barrier=True),0)
    for t in [.1,.2,1.]:assert b.step(obs(barrier_clear=True,front=3.5),t).v==0

def test_right_occupied_waits():
    b=Brain(3)
    for i in range(20):assert b.step(obs(intersection=True,right=.5,front=3.5,route_confirmed=True),i*.1).v==0

def test_unknown_route_waits():
    b=Brain(3)
    for i in range(20):assert b.step(obs(intersection=True,right=3.5,front=3.5),i*.1).v==0

def test_lidar_timeout():
    b=Brain(4);b.step(obs(front=2,left=.6,right=.6),0)
    assert b.state=='INSIDE'
    assert b.step(obs(front=2,left=.6,right=.6),60).state=='TIMEOUT'

def test_lidar_camera_forbidden():assert Brain(4).step(obs(camera_used=True),0).state=='FAULT'

def test_lidar_avoidance_direction():
    c=Brain(4).step(obs(front=.3,left=.5,right=.5,fl=.6,fr=.3,heading=.4),0)
    assert c.v==0 and c.w>0

def test_storage_requires_calibration():assert Brain(5).step(obs(source_id=1,pallet=True),0).v==0

def test_storage_holds_magnet_on_fault():
    b=Brain(5);b.step(obs(storage_calibrated=True,source_id=1,pallet=True,at_pickup=True),0)
    assert b.magnet
    b.step(obs(storage_calibrated=True,forbidden_zone=True),.1)
    assert b.state=='FAULT' and b.magnet

def test_storage_orientation_required():
    b=Brain(5);b.state='TRANSFER';b.magnet=True
    c=b.step(obs(storage_calibrated=True,target_id=2,path_allowed=True,at_drop=True,orientation_ok=False),0)
    assert c.v==0 and c.magnet

def test_guard_disarmed():assert guard(Command(.1,.2),CLEAR,False,True).v==0

def test_guard_stale():assert guard(Command(.1,.2),CLEAR,True,False).v==0

def test_guard_invalid():assert guard(Command(math.nan,.2),CLEAR,True,True).v==0

def test_guard_collision():assert guard(Command(.1,0),dict(CLEAR,front=.15),True,True).v==0

def test_guard_reverse():assert guard(Command(-.1,0),CLEAR,True,True).v==0

def test_guard_rotation_unknown_rear():assert guard(Command(0,.2),dict(CLEAR,rear=None),True,True).w==0

def test_guard_limits():
    c=guard(Command(.9,4),CLEAR,True,True);assert c.v==.1 and c.w==.7

def test_nan_scan_is_unknown():
    assert scan_features([math.nan]*360,-math.pi,math.pi/180)['front'] is None

def test_inf_scan_clear():
    assert scan_features([math.inf]*360,-math.pi,math.pi/180)['front']==3.5

def test_partial_scan_unknown_rear():
    assert scan_features([1.]*90,-math.pi/4,math.pi/180)['rear'] is None

def test_gap_rejects_point_inside_clearance():
    from urmc_core.control import gap_heading
    ranges=[math.inf]*360;ranges[180]=.4
    heading=gap_heading(ranges,-math.pi,math.pi/180)
    assert heading is not None and abs(heading)>.2


def test_no_gap_with_dense_close_ring():
    from urmc_core.control import gap_heading
    assert gap_heading([.12]*360,-math.pi,math.pi/180) is None
