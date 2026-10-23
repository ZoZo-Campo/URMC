"""Deterministic mission logic. SI units; no ROS, no hardware access."""
import math
from dataclasses import dataclass

@dataclass
class Command:
    v: float = 0.0
    w: float = 0.0
    state: str = 'WAIT'
    magnet: bool = False


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def sector(ranges, angle_min, increment, direction, half_width, rmin=.05, rmax=3.5):
    selected = [r for i, r in enumerate(ranges)
                if abs(wrap(angle_min+i*increment-direction)) <= half_width]
    valid = [rmax if r == math.inf else r for r in selected
             if r == math.inf or (math.isfinite(r) and rmin <= r <= rmax)]
    if not selected or len(valid)/len(selected) < .8:
        return None
    return min(valid)


def scan_features(ranges, amin, inc, rmin=.05, rmax=3.5, compute_heading=True):
    features = {name:sector(ranges, amin, inc, angle, width, rmin, rmax)
            for name, angle, width in [('front',0,.25),('left',math.pi/2,.25),
              ('right',-math.pi/2,.25),('rear',math.pi,.25),('fl',.65,.2),('fr',-.65,.2)]}
    features['heading']=gap_heading(ranges,amin,inc,rmin,rmax) if compute_heading else None
    return features


def gap_heading(ranges,amin,inc,rmin=.05,rmax=3.5):
    """Find a short straight corridor with 11 cm clearance from the lidar points.

    No map or odometry. This is a local avoidance heuristic, not a global planner.
    """
    points=[(r*math.cos(amin+i*inc),r*math.sin(amin+i*inc)) for i,r in enumerate(ranges)
            if math.isfinite(r) and rmin<=r<=min(rmax,1.2)]
    candidates=[i*.035 for i in range(-30,31)]
    free=[]
    for a in candidates:
        ca,sa=math.cos(a),math.sin(a)
        if not any(0<x*ca+y*sa<.6 and abs(-x*sa+y*ca)<.11 for x,y in points):free.append(a)
    if not free:return None
    nearest=min(free,key=abs)
    # A small inset avoids repeatedly choosing a corridor's exact boundary.
    if abs(nearest)>.01:
        inset=nearest+math.copysign(.07,nearest)
        if any(abs(a-inset)<.01 for a in free):nearest=inset
    return nearest


class BaseBrain:
    def __init__(self, mission, phase='checker', speed=.07, kp=.8):
        self.mission=int(mission); self.phase=phase; self.speed=speed; self.kp=kp
        self.state='WAIT'; self.clear_since=None; self.last_time=None
        self.entered=None; self.cross_start=None; self.finish_count=0
        self.last_sample=None; self.barrier_seen=False; self.magnet=False; self.grip_since=None

    def result(self,v=0.,w=0.):
        return Command(v,w,self.state,self.magnet)

    def stable(self, condition, now, delay=.6):
        if not condition:
            self.clear_since=None; return False
        if self.clear_since is None: self.clear_since=now
        return now-self.clear_since>=delay

    def lane(self,o):
        if not o.get('lane_valid'): return self.result()
        error=o.get('error',0.)
        if not math.isfinite(error): return self.result()
        return self.result(self.speed/(1+2*abs(error)),max(-.65,min(.65,-self.kp*error)))

    def step(self,o,now):
        if not math.isfinite(now): return self.result()
        if self.last_time is not None and (now<self.last_time or now-self.last_time>.35):
            self.clear_since=None; self.finish_count=0
        self.last_time=now
        if self.state in ('DONE','FAULT','TIMEOUT'): return self.result()
        if not o.get('fresh'):
            self.clear_since=None; self.finish_count=0; return self.result()
        if self.state=='WAIT': self.state='FOLLOW' if self.mission<=3 else 'ENTRY' if self.mission==4 else 'FIND_1'
        return self.decide(o,now)

    def navigate(self,o,name):
        goal=o.get(name+'_goal'); pose=o.get('pose')
        if not isinstance(goal,(list,tuple)) or len(goal)!=3 or not pose: return self.result()
        if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in [*goal,*pose]):return self.result()
        x,y,yaw=pose;gx,gy,gyaw=goal;distance=math.hypot(gx-x,gy-y)
        error=wrap(math.atan2(gy-y,gx-x)-yaw) if distance>.035 else wrap(gyaw-yaw)
        if distance<.035 and abs(error)<.08:return self.result()
        return self.result(min(self.speed,distance*.5) if distance>.035 and abs(error)<.35 else 0.,max(-.5,min(.5,error)))


def guard(command, scan, armed, fresh, max_v=.10, max_w=.7):
    if not armed or not fresh or scan is None:return Command(state=command.state,magnet=command.magnet)
    if not all(math.isfinite(v) for v in (command.v,command.w)):return Command(state='INVALID_COMMAND',magnet=command.magnet)
    # Reverse is intentionally unsupported. Rotation needs left/right/front coverage.
    if command.v<0:return Command(state='REVERSE_BLOCKED',magnet=command.magnet)
    if scan.get('front') is None or scan.get('front',0)<.16+command.v*.6:return Command(state=command.state,magnet=command.magnet)
    if abs(command.w)>.01 and any(scan.get(k) is None or scan[k]<.14 for k in ('left','right','rear')):
        return Command(state=command.state,magnet=command.magnet)
    return Command(min(max_v,command.v),max(-max_w,min(max_w,command.w)),command.state,command.magnet)


def Brain(mission,phase='checker',speed=.07,kp=.8):
    """Instantiate the logic shipped inside the selected mission package."""
    from importlib import import_module
    if int(mission) not in range(1,6):raise ValueError('Mission must be 1..5')
    module=import_module(f'urmc_mission_{int(mission):02d}.logic')
    return module.Mission(int(mission),phase,speed,kp)
