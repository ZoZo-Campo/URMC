from urmc_core.control import Brain, guard

CLEAR=dict(front=3.5,left=3.5,right=3.5,rear=3.5,fl=3.5,fr=3.5)
def observation(**kw):
 return dict(CLEAR,fresh=True,pose=(0.,0.,0.),**kw)

def test_search_rotates_without_advancing_and_times_out():
 b=Brain(1)
 c=b.step(observation(lane_valid=False),0)
 assert c.state=='SEARCH' and c.v==0 and c.w!=0
 for i in range(1,190):c=b.step(observation(lane_valid=False),i*.1)
 assert c.state=='TIMEOUT' and c.v==c.w==0

def test_search_requires_three_distinct_frames_to_resume():
 b=Brain(1);b.step(observation(lane_valid=False),0)
 for t in [.1,.2,.3]:assert b.step(observation(lane_valid=True,error=0,sample_id=1),t).v==0
 assert b.step(observation(lane_valid=True,error=0,sample_id=2),.4).v==0
 assert b.step(observation(lane_valid=True,error=0,sample_id=3),.5).v>0

def test_obstacle_without_odom_or_space_cannot_start_detour():
 for pose in (None,(0.,0.,0.)):
  o=observation(lane_valid=True,error=0);o.update(front=.3,left=.3,right=.3,pose=pose)
  c=Brain(1).step(o,0)
  assert c.state=='BLOCKED' and c.v==c.w==0

def test_close_object_stops_even_while_searching():
 o=observation(lane_valid=False);o['front']=.13
 c=Brain(1).step(o,0);assert c.state=='BLOCKED' and c.v==c.w==0

def test_guard_stops_search_on_stale_scan():
 c=Brain(1).step(observation(lane_valid=False),0)
 stopped=guard(c,CLEAR,True,False);assert stopped.v==stopped.w==0

def test_single_border_fallback_requires_prior_width():
 c=Brain(1).step(observation(lane_valid=False,yellow_x=100,white_x=None),0)
 assert c.v==0 and c.state=='SEARCH'
