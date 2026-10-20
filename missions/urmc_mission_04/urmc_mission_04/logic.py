"""Mission-specific decisions; ROS adapter and guards live in urmc_core."""
from urmc_core.control import BaseBrain

class Mission(BaseBrain):
    def decide(self,o,now):
        if o.get('camera_used'): self.state='FAULT';return self.result()
        if self.entered is not None and now-self.entered>=60: self.state='TIMEOUT';return self.result()
        f,l,r=o.get('front'),o.get('left'),o.get('right')
        if any(v is None for v in (f,l,r)): return self.result()
        if self.state=='ENTRY' and l<1 and r<1:
            self.state='INSIDE';self.entered=now
        if self.state=='INSIDE' and self.stable(l>1.2 and r>1.2 and f>1.5,now,.5):
            self.state='DONE';return self.result()
        heading=o.get('heading')
        if heading is None:
            return self.result(0.,.35 if o.get('fl',0)>=o.get('fr',0) else -.35)
        w=max(-.5,min(.5,1.4*heading))
        return self.result(self.speed if abs(heading)<.12 else 0.,w)
