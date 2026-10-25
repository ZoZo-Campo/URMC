"""Cross an unmarked intersection with right-of-way, fresh scans and odometry."""
import math
import pytest
from urmc_core.priority import PriorityBrain,priority_scan,RightTrafficTracker
from urmc_core.operator_settings import DEFAULTS,validate

CFG=dict(DEFAULTS,priority_clear_delay=.5)
def observation(t,**values):
    return dict(dict(fresh=True,lane_valid=True,error=0.,lane_width=150.,pose=(0.,0.,0.),
                     sample_id=round(t,3),scan_id=round(t,3),front=3.5,right=3.5,route_confirmed=True),**values)
def at_entry():
    b=PriorityBrain()
    for i in range(7):b.step(observation(i*.1),i*.1,CFG)
    for i in range(7,10):b.step(observation(i*.1,lane_valid=False,lane_width=None),i*.1,CFG)
    assert b.state=='YIELD';return b

def begin_crossing(b):
    for i in range(10,19):b.step(observation(i*.1,lane_valid=False),i*.1,CFG)
    assert b.state=='CROSS'

def test_line_loss_before_stable_approach_never_drives_blind():
    b=PriorityBrain()
    for i in range(20):assert b.step(observation(i*.1,lane_valid=False),i*.1,CFG).v==0
    assert b.state=='FOLLOW'

def test_yield_before_crossing_then_drive_without_lines():
    b=at_entry();begin_crossing(b)
    c=b.step(observation(1.9,lane_valid=False,pose=(.1,0.,0.)),1.9,CFG)
    assert c.v>0 and c.w==0 and c.state=='CROSS'

def test_duplicate_camera_cannot_confirm_entry():
    b=PriorityBrain()
    for i in range(7):b.step(observation(i*.1),i*.1,CFG)
    for i in range(7,11):b.step(observation(i*.1,lane_valid=False,sample_id='same'),i*.1,CFG)
    assert b.state=='FOLLOW'

def test_duplicate_scan_cannot_confirm_clear():
    b=at_entry()
    for i in range(10,30):assert b.step(observation(i*.1,lane_valid=False,scan_id='same'),i*.1,CFG).v==0
    assert b.state=='YIELD'

def test_priority_occupied_then_latched_stop_during_cross():
    b=at_entry()
    for i in range(10,20):assert b.step(observation(i*.1,right=.4),i*.1,CFG).v==0
    for i in range(20,30):b.step(observation(i*.1,lane_valid=False),i*.1,CFG)
    assert b.state=='CROSS'
    assert b.step(observation(3.,lane_valid=False,right=.4,pose=(.1,0.,0.)),3.,CFG).v==0

def test_stale_camera_is_not_an_intersection():
    b=PriorityBrain()
    for i in range(7):b.step(observation(i*.1),i*.1,CFG)
    for i in range(7,20):assert b.step(observation(i*.1,fresh=False,lane_valid=False),i*.1,CFG).v==0
    assert b.state=='FOLLOW'

def test_hold_heading_uses_yaw_and_projected_distance_not_world_x():
    b=PriorityBrain()
    for i in range(7):b.step(observation(i*.1,pose=(0.,0.,math.pi/2)),i*.1,CFG)
    for i in range(7,20):b.step(observation(i*.1,lane_valid=False,pose=(0.,0.,math.pi/2)),i*.1,CFG)
    c=b.step(observation(2.,lane_valid=False,pose=(0.,.2,math.pi/2+.1)),2.,CFG)
    assert c.w<0 and c.v>0 and b.progress==pytest.approx(.2)

def test_reacquire_requires_distance_and_three_distinct_images():
    b=at_entry();begin_crossing(b)
    for i in range(19,23):b.step(observation(i*.1,pose=(.1,0.,0.)),i*.1,CFG)
    assert b.state=='CROSS'
    for i in range(23,27):b.step(observation(i*.1,pose=(.35,0.,0.)),i*.1,CFG)
    assert b.state=='FOLLOW_AFTER_INTERSECTION'
    assert b.step(observation(2.7,lane_valid=False,pose=(.4,0.,0.)),2.7,CFG).v==0
    assert b.state=='FOLLOW_AFTER_INTERSECTION' # cannot blindly cross a second gap

def test_crossing_is_bounded_and_excess_lateral_error_stops():
    for pose in [(1.01,0.,0.),(.1,.13,0.),(.1,0.,.6)]:
        b=at_entry();begin_crossing(b)
        c=b.step(observation(1.9,lane_valid=False,pose=pose),1.9,CFG)
        assert c.v==0 and b.state=='FAULT'

def test_front_obstacle_and_unknown_scan_stop():
    for values in [dict(front=.2),dict(front=None),dict(right=None)]:
        b=at_entry();begin_crossing(b)
        assert b.step(observation(1.9,lane_valid=False,**values),1.9,CFG).v==0

def test_scan_unknown_is_not_clear_and_orientation_is_applied():
    r=[math.inf]*360;r[90]=.4
    assert priority_scan(r,-math.pi,math.pi/180)['right']==.4
    assert priority_scan([math.nan]*360,-math.pi,math.pi/180)['right'] is None
    r=[math.inf]*360;r[180]=.3
    assert priority_scan(r,-math.pi,math.pi/180,cfg=dict(DEFAULTS,scan_yaw_offset=-90))['right']==.3

def scan_of(x,y,width=.18):
    # A short horizontal robot contour in local coordinates, exact beam intersections.
    data=[]
    for i in range(720):
        a=-math.pi+i*2*math.pi/720
        if abs(math.sin(a))<1e-6:data.append(math.inf);continue
        r=y/math.sin(a);hit=r*math.cos(a)
        data.append(r if r>.05 and abs(hit-x)<width/2 else math.inf)
    return data

def update(tracker,x,y,t,**kw):
    return tracker.update(scan_of(x,y),-math.pi,2*math.pi/720,.05,3.5,kw.get('pose',(0.,0.,0.)),t,t,kw.get('cfg',DEFAULTS))

def test_stationary_person_outside_right_approach_is_not_priority():
    tracker=RightTrafficTracker()
    for i in range(15):r=update(tracker,-.6,-.4,i*.1)
    assert r['known'] and not r['blocked'] and not r['objects']

def test_optional_motion_filter_excludes_static_echo():
    tracker=RightTrafficTracker()
    for i in range(15):r=update(tracker,.2,-.5,i*.1,cfg=dict(DEFAULTS,priority_motion_required=1))
    assert r['known'] and not r['blocked'] and r['objects'][0]['label']=='STATIQUE / HORS GABARIT'

def test_approach_and_target_distance_from_robot_not_just_side_coordinate():
    tracker=RightTrafficTracker()
    for i in range(12):r=update(tracker,.2,-.65+i*.01,i*.1)
    assert r['blocked'] and r['objects'][0]['approaching']
    assert .5<r['objects'][0]['distance']<.7
    other=RightTrafficTracker()
    for i in range(12):r=update(other,.65,-.7+i*.002,i*.1)
    assert not r['blocked'] and not r['objects'] # Euclidean range exceeds .8 m

def test_candidate_remains_blocking_if_it_pauses_and_disappears_briefly():
    tracker=RightTrafficTracker()
    for i in range(12):r=update(tracker,.2,-.65+i*.01,i*.1)
    for i in range(12,18):r=update(tracker,.2,-.54,i*.1)
    assert r['blocked']
    r=tracker.update([math.inf]*720,-math.pi,2*math.pi/720,.05,3.5,(0,0,0),1.9,1.9)
    assert r['blocked']
    r=tracker.update([math.inf]*720,-math.pi,2*math.pi/720,.05,3.5,(0,0,0),2.3,2.3)
    assert not r['blocked']

def test_ego_motion_does_not_make_static_object_approach():
    tracker=RightTrafficTracker()
    for i in range(15):
        y=i*.005
        r=update(tracker,.2,-.5-y,i*.1,pose=(0.,y,0.),cfg=dict(DEFAULTS,priority_motion_required=1))
    assert not r['blocked']

def test_filtered_static_echo_does_not_block_yield_but_proximity_does():
    b=at_entry()
    for i in range(10,20):b.step(observation(i*.1,lane_valid=False,right=.5,traffic=dict(known=True,blocked=False)),i*.1,CFG)
    assert b.state=='CROSS'
    assert b.step(observation(2.,lane_valid=False,right=.16,traffic=dict(known=True,blocked=False)),2.,CFG).v==0

def test_invalid_priority_settings_rejected():
    for cfg in [dict(priority_right_start=-40,priority_right_end=-60),dict(priority_cross_min=.8,priority_cross_max=.4),dict(priority_size_min=.2,priority_size_max=.2)]:
        with pytest.raises(ValueError):validate(cfg)


def test_distance_mode_also_yields_to_robot_sized_stopped_target():
    tracker=RightTrafficTracker()
    for i in range(15):r=update(tracker,.2,-.5,i*.1)
    assert r['blocked'] and r['objects'][0]['label']=='CIBLE DANS ZONE'
    assert not r['objects'][0]['approaching']


def test_losing_only_one_border_does_not_authorize_blind_crossing():
    b=PriorityBrain()
    for i in range(7):b.step(observation(i*.1),i*.1,CFG)
    for i in range(7,30):
        c=b.step(observation(i*.1,lane_valid=False,lane_width=None,entry_eligible=False),i*.1,CFG)
        assert c.v==0 and b.state=='FOLLOW'
