"""Simulation-only controls, shared validation and pauseable mission time."""
import math

DEFAULTS=dict(camera_off=False,scan_off=False,front_blocked=False,hide_white=False,hide_lines=False,
              barrier_hold=False,right_occupied=False,obstacle=False,grip_failure=False,
              brightness=1.0,camera_latency_ms=0.0)

def validate_changes(changes):
    if not isinstance(changes,dict) or not changes:raise ValueError('Réglages attendus')
    out={}
    for key,value in changes.items():
        if key not in DEFAULTS:raise ValueError('Réglage inconnu : '+key)
        if isinstance(DEFAULTS[key],bool):
            if type(value) is not bool:raise ValueError('Booléen attendu : '+key)
        else:
            limits=(.15,1.5) if key=='brightness' else (0.,600.)
            if type(value) not in (int,float) or not math.isfinite(value) or not limits[0]<=value<=limits[1]:raise ValueError('Valeur hors limites : '+key)
            value=float(value)
        out[key]=value
    return out

class Session:
    def __init__(self,autostart=False):
        self.running=autostart;self.revision=0;self.epoch=0;self.elapsed=0.
    def action(self,name):
        if name not in ('start','pause','restart'):raise ValueError('Action inconnue')
        self.revision+=1
        self.running=name=='start'
        if name=='restart':self.epoch+=1;self.elapsed=0.
    def advance(self,dt):
        if self.running:self.elapsed+=max(0.,min(.1,dt))
    def snapshot(self):return dict(running=self.running,revision=self.revision,epoch=self.epoch,t=self.elapsed)
