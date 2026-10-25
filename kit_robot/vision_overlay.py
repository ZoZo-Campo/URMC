"""Full camera diagnostics: lane, all blue objects, areas and three masks."""
import cv2,numpy as np
from urmc_core.operator_settings import current
from urmc_core.barrier import blue_mask

def diagnostic_view(image,lane,barrier,robot_id=None,priority=None):
 out=cv2.resize(image,(320,240)).copy();cfg=current();hsv=cv2.cvtColor(out,cv2.COLOR_BGR2HSV)
 top,bottom=lane.get('roi',[156,230]);white_v=lane.get('white_v',185)
 yellow=cv2.inRange(hsv,(cfg['yellow_low'],90,65),(cfg['yellow_high'],255,255))
 white=cv2.inRange(hsv,(0,0,white_v),(179,cfg['white_s'],255))
 for mask in (yellow,white):mask[:top]=0;mask[bottom:]=0
 cv2.rectangle(out,(0,top),(319,bottom),(160,160,160),1);cv2.line(out,(160,70),(160,239),(255,255,0),1)
 for key,color,label in [('yellow',(0,255,255),'J'),('white',(255,255,255),'B')]:
  point=lane.get(key+'_center')
  if point is not None:
   point=tuple(round(v) for v in point);cv2.circle(out,point,5,color,2)
   cv2.putText(out,label,(min(302,point[0]+6),point[1]),0,.4,color,1)
 if lane.get('target_center') is not None and lane.get('error') is not None:
  center=tuple(round(v) for v in lane['target_center']);cv2.circle(out,center,5,(0,255,0),2);cv2.arrowedLine(out,(160,239),center,(0,255,0),2)
 for obj in reversed(barrier.get('objects',[])):
  x,y,w,h=obj['bbox'];selected=obj['index']==barrier.get('selected_index')
  color=(0,170,255) if selected else (160,130,90)
  cv2.rectangle(out,(x,y),(x+w,y+h),color,2 if selected else 1)
  text='B%d %.1f%%'%(obj['index'],obj['area_percent'])
  cv2.putText(out,text,(max(1,min(230,x)),min(220,max(44,y-4))),0,.34,color,1)
 if 'ellipse_axes' in barrier:
  c=tuple(round(v) for v in barrier['center']);axes=tuple(max(1,round(v/2)) for v in barrier['ellipse_axes'])
  cv2.ellipse(out,c,axes,barrier['axis_angle'],0,360,(0,170,255),1);cv2.circle(out,c,3,(0,170,255),-1)
 cv2.rectangle(out,(0,0),(319,36),(18,30,28),-1)
 label='PISTE : '+(str(round(lane['error'],2)) if lane.get('error') is not None else 'NON VALIDEE')
 label+=' J:%d B:%dpx'%(lane.get('areas',{}).get('yellow',0),lane.get('areas',{}).get('white',0))
 cv2.putText(out,label,(5,13),0,.35,(0,240,140),1)
 status={'CLOSED':'FERMEE','OPEN':'LEVEE','MOVING':'EN MOUVEMENT','UNKNOWN':'INCONNUE'}[barrier.get('position','UNKNOWN')]
 cv2.putText(out,'BARRIERE '+status+' B1 %.1f%% a=%s'%(barrier.get('area_percent',0),str(barrier.get('angle','?'))),(5,28),0,.33,(0,190,255),1)
 if robot_id is not None:
  cv2.rectangle(out,(0,226),(80,239),(18,30,28),-1);cv2.putText(out,'ROBOT '+str(robot_id),(5,236),0,.3,(240,240,240),1)
 out=cv2.resize(out,(480,360));panels=[]
 for mask,color,title in [(yellow,(0,230,255),'JAUNE'),(white,(240,240,240),'BLANC'),(blue_mask(image),(255,140,50),'BLEU / SURFACE')]:
  panel=np.zeros((240,320,3),np.uint8);panel[mask>0]=color
  if title.startswith('BLEU'):
   for obj in barrier.get('objects',[]):
    x,y,w,h=obj['bbox'];cv2.rectangle(panel,(x,y),(x+w,y+h),(0,220,255) if obj['index']==1 else (130,130,130),2)
    cv2.putText(panel,'B%d %dpx'%(obj['index'],obj['area']),(max(1,min(190,x)),max(48,min(220,y))),0,.45,(255,255,255),1)
  panel=cv2.resize(panel,(160,120));cv2.rectangle(panel,(0,0),(159,15),(18,30,28),-1)
  cv2.putText(panel,title,(4,11),0,.28,(240,240,240),1);panels.append(panel)
 result=np.hstack((out,np.vstack(panels)))
 if priority:priority_overlay(result,priority)
 return result


def priority_overlay(out,status):
 # Keep the lane masks visible; put a compact laser map over the lower camera.
 cv2.rectangle(out,(0,299),(332,359),(18,30,28),-1)
 cv2.putText(out,'PRIORITE '+str(status.get('state','?')),(6,313),0,.40,(0,220,255),1)
 def distance(key):
  value=status.get(key);return '?' if value is None else '%.2fm'%value
 cv2.putText(out,'Droite '+distance('right')+' Avant '+distance('front'),(6,330),0,.38,(240,240,240),1)
 cv2.putText(out,'Traversee %.2f / %.2fm'%(status.get('crossing_distance',0),status.get('crossing_max',1)),(6,347),0,.38,(240,240,240),1)
 cx,cy=402,291;scale=47
 cv2.rectangle(out,(334,210),(479,359),(18,30,28),-1)
 for r in (1,2):cv2.circle(out,(cx,cy),round(scale*r/2),(65,90,85),1)
 cv2.line(out,(cx,cy),(cx,cy-58),(180,180,180),1)
 for a,r in status.get('lidar_points',[]):
  if r<=1.4:cv2.circle(out,(round(cx-r*math.sin(a)*scale),round(cy-r*math.cos(a)*scale)),1,(210,210,210),-1)
 for obj in (status.get('traffic') or {}).get('objects',[]):
  point=(round(cx-obj['y']*scale),round(cy-obj['x']*scale));color=(30,60,255) if obj.get('blocking',obj['approaching']) else (0,200,255)
  cv2.circle(out,point,5,color,2);cv2.putText(out,'T%d %.2fm'%(obj['id'],obj['distance']),(337,348),0,.30,color,1)
 cv2.circle(out,(cx,cy),4,(0,240,160),-1);cv2.putText(out,'LIDAR 1m',(342,224),0,.30,(240,240,240),1)
