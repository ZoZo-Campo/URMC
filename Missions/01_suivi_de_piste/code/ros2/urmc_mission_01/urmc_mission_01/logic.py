"""Camera following, bounded search and local odometry bypass. No simulator map."""
import math
from urmc_core.control import BaseBrain, wrap

class Mission(BaseBrain):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.width=None;self.turn=1.;self.lost_since=None;self.one_since=None
        self.waypoints=[];self.bypass_since=None;self.return_yaw=None;self.bypass_yaw=0.;self.reacquired=0;self.seen_sample=None

    def decide(self,o,now):
        marker=o.get('checker') if self.phase=='checker' else o.get('stop')
        sample=o.get('sample_id',now)
        if sample!=self.last_sample:
            self.finish_count=self.finish_count+1 if marker else 0
            self.last_sample=sample
        if self.finish_count>=3:self.state='DONE';return self.result()
        # An object too close for rotation or an unknown surrounding sector means wait.
        sectors=[o.get(k) for k in ('front','left','right','rear','fl','fr')]
        if any(v is not None and v<.18 for v in sectors):
            self.state='BLOCKED';return self.result()
        if self.return_yaw is not None:
            pose=o.get('pose')
            if pose is None:return self.result()
            error=wrap(self.return_yaw-pose[2]);self.state='REJOIN'
            if abs(error)>.08:return self.result(0.,max(-.6,min(.6,1.5*error)))
            self.return_yaw=None;self.lost_since=None;self.reacquired=0
        if self.waypoints:
            if now-self.bypass_since>65:self.state='TIMEOUT';return self.result()
            pose=o.get('pose')
            if not pose:return self.result()
            gx,gy=self.waypoints[0];distance=math.hypot(gx-pose[0],gy-pose[1])
            if distance<.035:
                self.waypoints.pop(0)
                if not self.waypoints:
                    self.return_yaw=self.bypass_yaw;self.state='REJOIN';return self.result()
                gx,gy=self.waypoints[0]
            self.state='BYPASS'
            angle=wrap(math.atan2(gy-pose[1],gx-pose[0])-pose[2])
            # Re-check the actual scan throughout the detour; no blind timed drive.
            if o.get('front',0)<.25:return self.result()
            return self.result(min(self.speed,.055) if abs(angle)<.22 else 0.,max(-.6,min(.6,1.5*angle)))
        if o.get('front',3.5)<.42:
            pose=o.get('pose')
            left=min(o.get('left',0),o.get('fl',0));right=min(o.get('right',0),o.get('fr',0))
            if pose is None or max(left,right)<.48:self.state='BLOCKED';return self.result()
            side=1 if left>=right else -1
            x,y,a=pose;self.bypass_yaw=a
            self.waypoints=[(x+dx*math.cos(a)-dy*math.sin(a),y+dx*math.sin(a)+dy*math.cos(a)) for dx,dy in [(0,side*.32),(.72,side*.32),(.72,0)]]
            self.bypass_since=now;self.state='BYPASS';return self.result()
        valid=o.get('lane_valid',False);error=o.get('error',0.)
        if valid:
            if o.get('lane_width'):self.width=o['lane_width']
            self.one_since=None
        elif self.width is not None and ((o.get('yellow_x') is None)!=(o.get('white_x') is None)):
            if self.one_since is None:self.one_since=now
            if now-self.one_since<8:
                midpoint=o['yellow_x']+self.width/2 if o.get('yellow_x') is not None else o['white_x']-self.width/2
                error=(midpoint-160)/160;valid=True
        if valid and math.isfinite(error):
            if sample!=self.seen_sample:self.reacquired+=1;self.seen_sample=sample
            if self.lost_since is not None and self.reacquired<3:return self.result()
            self.lost_since=None;self.state='FOLLOW' if o.get('lane_valid') else 'ONE_BORDER'
            w=max(-.65,min(.65,-3*self.kp*error))
            if abs(w)>.03:self.turn=1 if w>0 else -1
            return self.result(self.speed/(1+2*abs(error))*(1 if o.get('lane_valid') else .6),w)
        self.reacquired=0
        if self.lost_since is None:self.lost_since=now
        elapsed=now-self.lost_since
        if elapsed>18:self.state='TIMEOUT';return self.result()
        self.state='SEARCH'
        # Look first toward the last curve, then sweep the other way, then widen.
        direction=self.turn if elapsed<3 or elapsed>=9 else -self.turn
        return self.result(0.,.4*direction)
