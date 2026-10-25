"""Right-of-way controller shared by the ROS simulator and the real camera node.

Laser angles: zero ahead, negative right. Missing data never means clear.
The crossing retains the approach heading and uses projected odometry, not time.
"""
import math
from .control import BaseBrain, sector, wrap
from .operator_settings import DEFAULTS


def priority_scan(ranges, angle_min, increment, rmin=.05, rmax=3.5, cfg=None):
    cfg=cfg or DEFAULTS
    if not ranges or not math.isfinite(increment) or increment<=0 or not math.isfinite(angle_min) or not math.isfinite(rmax) or rmax<=rmin:
        return dict(right=None,front=None,points=[])
    angle_min+=math.radians(cfg['scan_yaw_offset'])
    lo,hi=map(math.radians,(cfg['priority_right_start'],cfg['priority_right_end']))
    right=sector(ranges,angle_min,increment,(lo+hi)/2,(hi-lo)/2,rmin,rmax)
    front=sector(ranges,angle_min,increment,0,math.radians(25),rmin,rmax)
    points=[[round(wrap(angle_min+i*increment),3),round(r,3)] for i,r in enumerate(ranges)
            if i%max(1,len(ranges)//90)==0 and math.isfinite(r) and rmin<=r<=rmax]
    return dict(right=right,front=front,points=points)


class PriorityBrain(BaseBrain):
    def __init__(self,mission=3,phase='checker',speed=.05,kp=.8,finish_after_cross=False):
        super().__init__(mission,phase,speed,kp)
        self.finish_after_cross=finish_after_cross
        self.origin=None;self.heading=None;self.last_good_pose=None;self.last_good_time=None
        self.good_frames=0;self.entry_frames=0;self.reacquire_frames=0;self.lane_width=None
        self.image_id=None;self.clear_scan=None;self.clear_frames=0;self.last_step=None
        self.progress=0.;self.lateral=0.;self.reason='Attente des capteurs';self.telemetry={}
        self.handled=False

    def lane(self,o):
        error=o.get('error',0.)
        return self.result(self.speed/(1+3*abs(error)),max(-.2,min(.2,-self.kp*error)))

    def reset_clear(self):
        self.clear_since=None;self.clear_scan=None;self.clear_frames=0

    def step(self,o,now,cfg=None):
        cfg=dict(DEFAULTS,**(cfg or {}));self.speed=cfg['speed'];self.kp=cfg['steering_gain']
        if self.state in ('DONE','FAULT'):return self.result()
        gap=self.last_step is not None and (now<self.last_step or now-self.last_step>.35)
        self.last_step=now
        if gap or not o.get('fresh'):
            self.reset_clear();self.good_frames=0;self.entry_frames=0;self.reacquire_frames=0
            if not o.get('fresh'):
                self.reason=o.get('stale_reason','Capteurs absents ou perimes');return self.result()
        pose=o.get('pose')
        if not pose or len(pose)!=3 or not all(math.isfinite(v) for v in pose):
            self.reason='Odometrique indisponible';self.reset_clear();return self.result()
        front=o.get('front');right=o.get('right')
        if front is None or right is None or not all(math.isfinite(v) and v>0 for v in (front,right)):
            self.reason='Secteurs LiDAR inconnus';self.reset_clear();return self.result()
        if self.state=='WAIT':self.state='FOLLOW'
        new_image=o.get('sample_id') is not None and o['sample_id']!=self.image_id
        if new_image:self.image_id=o['sample_id']
        lane=bool(o.get('lane_valid') and isinstance(o.get('error'),(float,int)) and math.isfinite(o['error']) and abs(o['error'])<=.65)
        width=o.get('lane_width')
        wide=bool(width and self.lane_width and width>1.4*self.lane_width)
        cue=bool(o.get('intersection') or wide)
        if self.state=='FOLLOW' and not self.handled:
            if new_image:
                trusted=lane and abs(o['error'])<.20 and not cue
                if trusted:
                    self.good_frames+=1;self.entry_frames=0
                    if self.good_frames>=6:
                        self.last_good_pose=tuple(pose);self.last_good_time=now
                    if width:self.lane_width=width if self.lane_width is None else .9*self.lane_width+.1*width
                elif (cue or (not lane and o.get('entry_eligible',True))) and self.last_good_time is not None and 0<=now-self.last_good_time<=.6:
                    self.entry_frames+=1
                else:self.good_frames=0;self.entry_frames=0
                if self.entry_frames>=3:
                    self.state='YIELD';self.origin=tuple(pose[:2]);self.heading=self.last_good_pose[2]
                    self.reset_clear()
            if self.state=='FOLLOW':
                if not lane or self.entry_frames:self.reason='Entree de carrefour a confirmer';return self.result()
                if front<=cfg['priority_front_stop']:self.reason='Obstacle devant';return self.result()
                self.reason='Suivi des deux bordures';return self.lane(o)
        if self.state in ('YIELD','CROSS'):
            if self.origin is None or self.heading is None:self.state='FAULT';self.reason='Cap de traversee absent';return self.result()
            dx,dy=pose[0]-self.origin[0],pose[1]-self.origin[1]
            self.progress=dx*math.cos(self.heading)+dy*math.sin(self.heading)
            self.lateral=-dx*math.sin(self.heading)+dy*math.cos(self.heading)
            if abs(self.lateral)>.12 or self.progress<-.06:
                self.state='FAULT';self.reason='Ecart a la trajectoire de traversee';return self.result()
            if self.state=='CROSS' and self.progress>=cfg['priority_cross_max']:
                self.state='FAULT';self.reason='Distance sans bordures depassee';return self.result()
            if not o.get('route_confirmed'):
                self.reason='Trajet tout droit non confirme';self.reset_clear();return self.result()
            traffic=o.get('traffic')
            if traffic is not None and not traffic.get('known'):
                self.reason='Analyse du mouvement a droite';self.reset_clear();return self.result()
            right_blocked=bool(traffic.get('blocked')) if traffic is not None else right<=cfg['priority_right_distance']
            blocked=right_blocked or right<=.18 or front<=cfg['priority_front_stop']
            if blocked:
                self.reason='Priorite au robot a droite' if right_blocked else 'Obstacle trop proche' if right<=.18 else 'Obstacle devant'
                self.reset_clear();self.reacquire_frames=0;return self.result()
            scan_id=o.get('scan_id')
            if scan_id is not None and scan_id!=self.clear_scan:
                self.clear_scan=scan_id;self.clear_frames+=1
                if self.clear_since is None:self.clear_since=now
            clear=self.clear_frames>=3 and self.clear_since is not None and now-self.clear_since>=cfg['priority_clear_delay']
            if not clear:
                self.reason='Confirmation droite libre';return self.result()
            if self.state=='YIELD':self.state='CROSS'
            if new_image:
                self.reacquire_frames=self.reacquire_frames+1 if lane and abs(o['error'])<.35 and not cue else 0
            if self.progress>=cfg['priority_cross_min'] and self.reacquire_frames>=3:
                self.handled=True;self.state='DONE' if self.finish_after_cross else 'FOLLOW_AFTER_INTERSECTION'
                self.reason='Deux bordures retrouvees'
                return self.result()  # One zero cycle before returning to visual steering.
            heading_error=wrap(self.heading-pose[2])
            if abs(heading_error)>.5:
                self.state='FAULT';self.reason='Cap de traversee perdu';return self.result()
            self.reason='Tout droit au cap memorise, recherche des deux bordures'
            w=max(-.25,min(.25,cfg['priority_heading_gain']*heading_error-1.2*self.lateral))
            return self.result(self.speed*.75/(1+2*abs(heading_error)),w)
        if self.state=='FOLLOW_AFTER_INTERSECTION':
            if not lane:self.reason='Bordures absentes apres intersection';return self.result()
            if front<=cfg['priority_front_stop']:self.reason='Obstacle devant';return self.result()
            self.reason='Suivi repris apres intersection';return self.lane(o)
        return self.result()


class RightTrafficTracker:
    """Geometric approaching-object candidates, never a semantic person classifier.

    Cluster association uses world positions to compensate for our own motion.
    Outside-road returns do not claim priority. Distance mode includes a stopped
    robot-sized object; optional motion mode requires an observed approach.
    A candidate already seen approaching remains latched while in the right lane.
    """
    def __init__(self):
        self.tracks={};self.next_id=1;self.previous_scan=None;self.result=dict(blocked=False,known=False,objects=[])
    def update(self,ranges,amin,inc,rmin,rmax,pose,now,scan_id,cfg=None):
        cfg=cfg or DEFAULTS
        if scan_id==self.previous_scan:return self.result
        self.previous_scan=scan_id
        if not pose or not ranges or inc<=0:
            self.result=dict(blocked=False,known=False,objects=[]);return self.result
        # Break at invalid beams and discontinuities; do not subsample for clustering.
        clusters=[];group=[];last=None
        for i,r in enumerate(ranges):
            a=amin+i*inc+math.radians(cfg['scan_yaw_offset'])
            point=(r*math.cos(a),r*math.sin(a)) if math.isfinite(r) and rmin<=r<=rmax else None
            if point is None or (last and math.hypot(point[0]-last[0],point[1]-last[1])>.09):
                if len(group)>=3:clusters.append(group)
                group=[]
            if point:group.append(point)
            last=point
        if len(group)>=3:clusters.append(group)
        objects=[];used=set();ca,sa=math.cos(pose[2]),math.sin(pose[2])
        for points in clusters:
            x=sum(p[0] for p in points)/len(points);y=sum(p[1] for p in points)/len(points)
            size=math.hypot(max(p[0] for p in points)-min(p[0] for p in points),max(p[1] for p in points)-min(p[1] for p in points))
            distance=min(math.hypot(p[0],p[1]) for p in points)
            angle=math.degrees(math.atan2(y,x))
            if not (-.15<x<cfg['priority_lane_forward'] and y<-.12 and distance<=cfg['priority_right_distance'] and cfg['priority_right_start']<=angle<=cfg['priority_right_end']):continue
            wx,wy=pose[0]+x*ca-y*sa,pose[1]+x*sa+y*ca
            available=[(math.hypot(wx-t['wx'],wy-t['wy']),key) for key,t in self.tracks.items()
                       if key not in used and 0<=now-t['seen']<.5]
            nearest=min(available,default=(math.inf,None))
            key=nearest[1] if nearest[0]<.20 else self.next_id
            if key==self.next_id:self.next_id+=1
            track=self.tracks.get(key)
            if track is None:track=dict(wx=wx,wy=wy,seen=now,start=now,history=[],approaching=False)
            history=[h for h in track['history'] if now-h[0]<=1.5]
            history.append((now,wx,wy));track.update(history=history,wx=wx,wy=wy,seen=now)
            vy=vx=0.
            if history and now-history[0][0]>=.35:
                dt=now-history[0][0];dx=(wx-history[0][1])/dt;dy=(wy-history[0][2])/dt
                vx=dx*ca+dy*sa;vy=-dx*sa+dy*ca
            size_ok=cfg['priority_size_min']<=size<=cfg['priority_size_max']
            displacement=(wy-history[0][2])*ca-(wx-history[0][1])*sa
            moving=size_ok and len(history)>=4 and displacement>=.025 and vy>=cfg['priority_min_approach_speed'] and vy>abs(vx)*.5
            track['approaching']=track['approaching'] or moving
            track['size_frames']=track.get('size_frames',0)+1 if size_ok else 0
            qualified=track['size_frames']>=4 and now-track['start']>=.4
            track['blocking']=track['approaching'] or (qualified and not cfg['priority_motion_required'])
            pending=not track['approaching'] and now-track['start']<.7
            label='ARRIVANT DE DROITE' if track['approaching'] else 'CIBLE DANS ZONE' if track['blocking'] else 'CONFIRMATION' if pending else 'STATIQUE / HORS GABARIT'
            obj=dict(id=key,x=round(x,3),y=round(y,3),distance=round(distance,3),size=round(size,3),vy=round(vy,3),vx=round(vx,3),
                     approaching=track['approaching'],blocking=track['blocking'],pending=pending,label=label)
            objects.append(obj);used.add(key);self.tracks[key]=track
        # Brief occlusions retain the approach candidate rather than releasing instantly.
        recent_hidden=any(t.get('blocking',False) and key not in used and 0<=now-t['seen']<.5 for key,t in self.tracks.items())
        self.tracks={key:t for key,t in self.tracks.items() if 0<=now-t['seen']<.5}
        blocked=recent_hidden or any(o['blocking'] for o in objects)
        self.result=dict(blocked=blocked,known=not any(o['pending'] for o in objects),objects=objects,
                         reason='Cible a droite dans le seuil de distance' if blocked else 'Analyse du mouvement' if any(o['pending'] for o in objects) else 'Aucun objet approchant dans la voie de droite')
        return self.result
