"""Blue barrier: observe closure, confirm vertical opening, then cross."""
from urmc_core.control import BaseBrain
from urmc_core.barrier import OpeningGate

class Mission(BaseBrain):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.opening=OpeningGate(delay=.6)
    def decide(self,o,now):
        position='CLOSED' if o.get('barrier') else 'OPEN' if o.get('barrier_clear') else 'UNKNOWN'
        ready=self.opening.update(dict(position=position),now,o.get('sample_id',now))
        if position=='CLOSED':
            self.barrier_seen=True;self.state='WAIT_BARRIER';return self.result()
        if not ready or o.get('front',0)<=.5:
            self.state='WAIT_BARRIER' if self.barrier_seen else 'WAIT_VISIBLE';return self.result()
        if self.cross_start is None:self.cross_start=o.get('x')
        self.state='PASS_BARRIER'
        if self.cross_start is not None and o.get('x',self.cross_start)-self.cross_start>.55:
            self.state='DONE';return self.result()
        return self.lane(o)
