"""Closed-loop M1 checks: pixels + scan + odometry, no map input to controller."""
import math,json
from urmc_core.sim import scene,lidar
from urmc_core.vision import analyze
from urmc_core.control import Brain,scan_features,guard
from urmc_core.course import center

def run(obstacle=False,loss=False,one=False):
 b=Brain(1);x=y=yaw=0.;history=[];worst=0.;clearance=99
 for k in range(6000):
  t=k*.05
  image=scene(x,y,yaw,1,t,hide_white=one and 5<t<8,hide_lines=loss and 5<t<7)
  o=analyze(image);scan=scan_features(lidar(x,y,yaw,1,t,obstacle=obstacle),-math.pi,math.pi/180,compute_heading=False)
  o.update(scan);o.update(fresh=True,pose=(x,y,yaw),sample_id=k)
  c=guard(b.step(o,t),scan,True,True)
  if not history or history[-1]!=c.state:history.append(c.state)
  if obstacle:
   distance=math.hypot(max(.48-x,0,x-.59),max(-.055-y,0,y-.055));clearance=min(clearance,distance)
   assert distance>.09,('collision',x,y,history)
  if c.state in ('FOLLOW','ONE_BORDER'):worst=max(worst,abs(y-center(x)))
  x+=c.v*math.cos(yaw)*.05;y+=c.v*math.sin(yaw)*.05;yaw+=c.w*.05
  if c.state in ('DONE','TIMEOUT','FAULT'):break
 result=dict(obstacle=obstacle,loss=loss,one=one,state=c.state,t=t,x=x,y=y,yaw=yaw,history=history,max_tracking_error=worst,clearance=clearance)
 print(json.dumps(result),flush=True)
 assert c.state=='DONE',result
 assert worst<.10,result
 if loss:assert 'SEARCH' in history
 if one:assert 'ONE_BORDER' in history
 if obstacle:assert 'BYPASS' in history
 return result
if __name__=='__main__':
 for args in ({},{'loss':True},{'one':True},{'obstacle':True}):run(**args)
