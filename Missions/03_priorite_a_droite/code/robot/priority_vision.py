"""Near/far lane views for mission 3 ONLY; far lines never confirm crossing exit."""
import cv2,numpy as np
from suivi_piste_camera import LaneTracker,lane_details
from urmc_core.operator_settings import DEFAULTS

class PriorityVision:
 def __init__(self):
  self.near_tracker=LaneTracker();self.far_tracker=LaneTracker();self.last_id=None;self.far_count=0;self.key=None;self.result={}
 def observe(self,data,sample_id,cfg=None,image=None):
  cfg=dict(DEFAULTS,**(cfg or {}));key=(cfg['priority_far_top'],cfg['priority_far_bottom'])
  if key!=self.key:self.far_tracker=LaneTracker();self.far_count=0;self.key=key
  if sample_id==self.last_id:return self.result
  if isinstance(sample_id,(int,float)) and isinstance(self.last_id,(int,float)) and not 0<sample_id-self.last_id<.35:self.far_count=0
  self.last_id=sample_id
  if image is None:image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
  near=lane_details(data,self.near_tracker,regions=((156,230),),image=image)
  far=lane_details(data,self.far_tracker,regions=((round(key[0]*2.4),round(key[1]*2.4)),),min_area=20,min_height=10,image=image)
  self.far_count=self.far_count+1 if far.get('error') is not None and abs(far['error'])<=.5 else 0
  self.result=dict(near=near,far=far,far_confirmed=self.far_count>=3)
  return self.result
