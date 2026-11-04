"""Controller freshness is checked at every tick, not frozen at image receipt."""
import sys,time
from pathlib import Path
import pytest,rclpy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from mission_barriere import BarrierNode
from suivi_piste_camera import Trial

@pytest.mark.parametrize('classic',[False,True])
def test_old_capture_cannot_drive_even_with_recent_receipt(classic,monkeypatch):
 rclpy.init(args=['--ros-args','-r','/cmd_vel:=/urmc_test/fresh_finish/cmd_vel'])
 actor=Trial() if classic else BarrierNode(True,0.,True)
 try:
  monkeypatch.setattr(actor,'count_publishers',lambda _:1)
  old=actor.get_clock().now().nanoseconds/1e9-.32;stamp=(int(old),int((old-int(old))*1e9))
  actor.received=time.monotonic();actor.error=0.;actor.created-=2
  if classic:
   actor.started=time.monotonic();actor.header_age=.01;actor.last_stamp=stamp;actor.tick();assert actor.done
  else:
   actor.previous_stamp=stamp;actor.image_age=.01;actor.lane_valid=5;actor.odom_received=time.monotonic();actor.tick();assert actor.state=='WAIT_CAMERA' and actor.v==0
 finally:actor.destroy_node();rclpy.shutdown()
