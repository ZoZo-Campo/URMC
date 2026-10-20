"""Mission-specific decisions; ROS adapter and guards live in urmc_core."""
from urmc_core.control import BaseBrain

class Mission(BaseBrain):
    def decide(self,o,now):
        if o.get('barrier'):
            self.barrier_seen=True; self.state='WAIT_BARRIER';self.clear_since=None
            return self.result()
        if self.state=='WAIT_BARRIER':
            if not self.stable(o.get('barrier_clear') and o.get('front',0)>.5,now): return self.result()
            self.state='PASS_BARRIER';self.cross_start=o.get('x')
        if self.state=='PASS_BARRIER' and self.cross_start is not None and o.get('x',self.cross_start)-self.cross_start>.55:
            self.state='DONE';return self.result()
        return self.lane(o)
        return self.result()
