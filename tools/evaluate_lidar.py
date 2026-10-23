"""Fast replay of same scan model/controller, including an offset and obstacle."""
import json,math
from urmc_core.sim import lidar
from urmc_core.control import Brain,guard,scan_features

def run(offset=0.,obstacle=False):
    x,y,yaw=.2,offset,0.;b=Brain(4);trace=[];collision=False
    segments=[(.6,-.6,2.,-.6),(.6,.6,2.,.6),(.6,-.6,.6,-.25),(.6,.25,.6,.6),(2.,-.6,2.,-.25),(2.,.25,2.,.6)]
    if obstacle:segments += [(1.25,.1,1.4,.1),(1.25,.25,1.4,.25),(1.25,.1,1.25,.25),(1.4,.1,1.4,.25)]
    for i in range(1600):
        t=i*.05;scan=scan_features(lidar(x,y,yaw,4,t,obstacle),-math.pi,math.pi/180)
        cmd=guard(b.step(dict(scan,fresh=True),t),scan,True,True)
        x+=cmd.v*math.cos(yaw)*.05;y+=cmd.v*math.sin(yaw)*.05;yaw+=cmd.w*.05
        for ax,ay,bx,by in segments:
            u=max(0,min(1,((x-ax)*(bx-ax)+(y-ay)*(by-ay))/((bx-ax)**2+(by-ay)**2)))
            collision |= math.hypot(x-ax-u*(bx-ax),y-ay-u*(by-ay))<.069
        if b.state in ('DONE','TIMEOUT','FAULT'):break
    return dict(offset=offset,obstacle=obstacle,state=b.state,t=t,x=x,y=y,collision=collision)

if __name__=='__main__':
    results=[run(y,o) for y,o in [(0,False),(.04,False),(-.04,False),(0,True)]]
    print(json.dumps(results,indent=2))
