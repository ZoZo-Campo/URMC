"""Validated settings shared by the operator console and camera algorithms."""
import json,math,os,time
from pathlib import Path

# default, minimum, maximum, step, label, group
SCHEMA={
 'speed':(.05,.01,.08,.005,'Vitesse maximale (m/s)','Conduite'),
 'steering_gain':(.8,.2,2.,.1,'Correction de direction','Conduite'),
 'yellow_low':(18,0,178,1,'Jaune : teinte minimum','Piste'),
 'yellow_high':(38,1,179,1,'Jaune : teinte maximum','Piste'),
 'white_s':(65,10,110,1,'Blanc : saturation maximum','Piste'),
 'white_floor':(95,50,170,1,'Blanc : luminosité minimale','Piste'),
 'blue_low':(100,0,178,1,'Bleu : teinte minimum','Barrière'),
 'blue_high':(130,1,179,1,'Bleu : teinte maximum','Barrière'),
 'blue_s':(100,30,220,1,'Bleu : saturation minimum','Barrière'),
 'blue_area_min':(100,30,4000,10,'Bleu : surface minimale (pixels)','Barrière'),
 'blue_dominance':(1.4,1.1,3.,.1,'Bleu : dominance de la plus grande surface','Barrière'),
 'blue_v':(50,20,180,1,'Bleu : luminosité minimum','Barrière'),
 'closed_angle':(25,5,40,1,'Angle fermé maximum (°)','Barrière'),
 'open_angle':(65,50,85,1,'Angle levé minimum (°)','Barrière'),
 'scan_yaw_offset':(0,-180,180,1,'Orientation du LiDAR (°)','Priorité à droite'),
 'priority_right_start':(-120,-170,-40,1,'Secteur droit : début (°)','Priorité à droite'),
 'priority_right_end':(-20,-60,-5,1,'Secteur droit : fin (°)','Priorité à droite'),
 'priority_right_distance':(.8,.3,2.,.05,'Priorité : distance objet–robot maximum (m)','Priorité à droite'),
 'priority_front_stop':(.30,.18,.6,.01,'Arrêt obstacle devant (m)','Priorité à droite'),
 'priority_lane_forward':(.75,.25,1.5,.05,'Voie de droite : limite devant (m)','Priorité à droite'),
 'priority_size_min':(.08,.04,.2,.01,'Objet attendu : largeur minimale (m)','Priorité à droite'),
 'priority_size_max':(.40,.2,.8,.01,'Objet attendu : largeur maximale (m)','Priorité à droite'),
 'priority_motion_required':(0,0,1,1,'Exiger une approche observée (0 non / 1 oui)','Priorité à droite'),
 'priority_min_approach_speed':(.025,.01,.2,.005,'Mouvement vers le carrefour minimum (m/s)','Priorité à droite'),
 'priority_clear_delay':(1.,.5,3.,.1,'Confirmation droite libre (s)','Priorité à droite'),
 'priority_cross_min':(.30,.1,.8,.05,'Reprise des lignes après au moins (m)','Priorité à droite'),
 'priority_cross_max':(1.,.4,1.8,.05,'Distance maximale sans lignes (m)','Priorité à droite'),
 'priority_heading_gain':(1.5,.5,3.,.1,'Correction du cap sans lignes','Priorité à droite'),
 'm4_speed':(.10,.02,.15,.01,'Vitesse mission 4 (m/s)','Enceinte / SLAM'),
 'm4_radius':(.105,.09,.16,.005,'Rayon physique du robot (m)','Enceinte / SLAM'),
 'm4_margin':(.025,.01,.06,.005,'Marge autour du robot (m)','Enceinte / SLAM'),
 'm4_camera_light':(15,5,60,1,'Caméra d’essai : luminosité minimale','Enceinte / SLAM'),
 'm4_camera_change':(.12,.05,.4,.01,'Caméra d’essai : seuil de changement à l’arrêt','Enceinte / SLAM'),
 'm4_goal_tolerance':(.07,.04,.12,.01,'Tolérance aux repères (m)','Enceinte / SLAM'),
 'opening_delay':(1.,.5,3.,.1,'Confirmation de levée (s)','Barrière'),
}
DEFAULTS={key:value[0] for key,value in SCHEMA.items()}

def validate(values):
 if not isinstance(values,dict) or set(values)-set(SCHEMA):raise ValueError('Réglage inconnu.')
 result=dict(DEFAULTS)
 for key,value in values.items():
  if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):raise ValueError('Valeur numérique attendue : '+key)
  default,lo,hi,*_=SCHEMA[key]
  if not lo<=value<=hi:raise ValueError(f'{key} doit être entre {lo} et {hi}.')
  if isinstance(default,int) and int(value)!=value:raise ValueError('Valeur entière attendue : '+key)
  result[key]=int(value) if isinstance(default,int) else float(value)
 if result['yellow_low']>=result['yellow_high'] or result['blue_low']>=result['blue_high']:raise ValueError('La teinte minimum doit être inférieure au maximum.')
 if result['priority_size_min']>=result['priority_size_max']:raise ValueError('Le gabarit minimal doit être inférieur au maximal.')
 if result['priority_right_start']>=result['priority_right_end']:raise ValueError('Le début du secteur droit doit précéder sa fin.')
 if result['priority_cross_min']>=result['priority_cross_max']:raise ValueError('La distance minimale doit être inférieure à la distance maximale.')
 return result

_cached_path=None;_cached=dict(DEFAULTS);_checked=0.;_stamp=None

def current():
 global _cached_path,_cached,_checked,_stamp
 path=os.environ.get('URMC_SETTINGS_FILE')
 if not path:return dict(DEFAULTS)
 now=time.monotonic()
 if path==_cached_path and now-_checked<.3:return dict(_cached)
 _checked=now
 try:
  stat=Path(path).stat();stamp=(stat.st_mtime_ns,stat.st_size,stat.st_ino)
  if path!=_cached_path or stamp!=_stamp:
   _cached=validate(json.loads(Path(path).read_text()));_stamp=stamp;_cached_path=path
 except (OSError,ValueError,TypeError):
  # A broken settings file must not silently retain potentially faster settings.
  _cached=dict(DEFAULTS);_cached["speed"]=0.;_cached_path=path;_stamp=None
 return dict(_cached)
