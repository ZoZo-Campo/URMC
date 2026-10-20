"""Mission-specific decisions; ROS adapter and guards live in urmc_core."""
from urmc_core.control import BaseBrain

class Mission(BaseBrain):
    def decide(self,o,now):
        if not o.get('storage_calibrated'): return self.result()
        if o.get('forbidden_zone'): self.state='FAULT';return self.result()
        if self.state=='FIND_1':
            if o.get('source_id')!=1 or not o.get('pallet'): return self.result()
            self.state='APPROACH'
        if self.state=='APPROACH':
            if not o.get('at_pickup'): return self.navigate(o,'pickup')
            self.state='GRIP';self.magnet=True
        if self.state=='GRIP':
            if not o.get('grip_confirmed'): return self.result()
            self.state='TRANSFER'
        if self.state=='TRANSFER':
            if not o.get('grip_confirmed'):return self.result()
            if o.get('target_id')!=2 or not o.get('path_allowed'): return self.result()
            if not o.get('at_drop'): return self.navigate(o,'drop')
            if not o.get('orientation_ok'): return self.result()
            self.state='RELEASE';self.magnet=False
        if self.state=='RELEASE':
            if not o.get('released_confirmed'): return self.result()
            self.state='EXIT'
        if self.state=='EXIT':
            if o.get('zone_exited'): self.state='DONE';return self.result()
            return self.navigate(o,'exit')
        return self.result()
