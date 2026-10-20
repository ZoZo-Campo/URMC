"""Mission-specific decisions; ROS adapter and guards live in urmc_core."""
from urmc_core.control import BaseBrain

class Mission(BaseBrain):
    def decide(self,o,now):
        if self.state=='FOLLOW' and o.get('intersection'): self.state='YIELD'
        if self.state=='YIELD':
            if not self.stable(o.get('right',0)>1. and o.get('front',0)>.5 and o.get('route_confirmed'),now,.8): return self.result()
            self.state='CROSS';self.cross_start=o.get('x')
        if self.state=='CROSS':
            if not o.get('route_confirmed') or o.get('front',0)<.3: return self.result()
            if self.cross_start is not None and o.get('x',self.cross_start)-self.cross_start>.6:
                self.state='DONE';return self.result()
        return self.lane(o)
        return self.result()
