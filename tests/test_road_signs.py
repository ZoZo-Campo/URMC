import math
import cv2,numpy as np,pytest
from urmc_core.road_signs import detect_stop,VisualConfirmation,FinishGate
from urmc_core.tunnel_sign import detect_tunnel
from urmc_core.operator_settings import DEFAULTS

def sign(kind='stop',gain=1.,shift=0):
 image=np.full((240,320,3),60,np.uint8);c=(240+shift,90);r=26
 if kind=='no-entry':
  cv2.circle(image,c,r,(0,0,230),-1);cv2.rectangle(image,(c[0]-18,85),(c[0]+18,95),(245,245,245),-1)
 elif kind=='stop':
  points=np.array([(c[0]+r*math.cos(a),c[1]+r*math.sin(a)) for a in np.arange(8)*math.pi/4+math.pi/8],np.int32)
  cv2.fillPoly(image,[points],(0,0,230));cv2.putText(image,'STOP',(c[0]-20,95),0,.25,(245,245,245),1)
 elif kind=='red-circle':cv2.circle(image,c,r,(0,0,230),-1)
 elif kind=='red-box':
  cv2.rectangle(image,(210,65),(265,115),(0,0,230),-1);cv2.line(image,(215,90),(260,90),(245,245,245),8)
 elif kind=='blue':cv2.rectangle(image,(20,80),(300,105),(230,70,20),-1)
 image=np.clip(image.astype(float)*gain,0,255).astype(np.uint8)
 return cv2.imdecode(cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,55])[1],1)

@pytest.mark.parametrize('kind',['stop','no-entry'])
@pytest.mark.parametrize('gain',[.6,.85,1.,1.2])
def test_stop_requires_red_shape_and_white_center(kind,gain):assert detect_stop(sign(kind,gain))['visible']
@pytest.mark.parametrize('kind',['red-circle','red-box','blue'])
def test_objects_without_a_supported_sign_do_not_stop(kind):assert not detect_stop(sign(kind))['visible']

def test_distinct_fresh_coherent_images_are_required():
 gate=FinishGate('stop');im=sign()
 assert gate.update(im,False,0,1) is None
 for t in [.1,.2,.3]:assert gate.update(im,False,t,1) is None
 assert gate.update(im,False,.4,2) is None # long gap breaks confirmation
 assert gate.update(im,False,.5,3) is None
 assert gate.update(im,False,.6,4)['kind']=='stop'
 assert gate.update(sign('blue'),False,10,5)['kind']=='stop' # finish is latched

def test_an_old_image_cannot_contribute_to_confirmation():
 gate=FinishGate('stop')
 for now,stamp in [(0,3),(.1,2),(.2,1)]:assert gate.update(sign(),False,now,stamp) is None
 assert gate.stop.frames==1

def test_marker_choice_does_not_stop_for_the_other_finish():
 stop=FinishGate('stop');checker=FinishGate('checker')
 for i in range(5):
  assert stop.update(sign('blue'),True,i*.1,i) is None
  assert checker.update(sign(),False,i*.1,i) is None
 assert checker.update(sign('blue'),True,.5,5) is None
 assert checker.update(sign('blue'),True,.6,6) is None
 assert checker.update(sign('blue'),True,.7,7)['kind']=='checker'

def test_spatially_unrelated_red_candidates_cannot_accumulate():
 gate=FinishGate('stop')
 for i in range(6):assert gate.update(sign(shift=-100 if i%2 else 0),False,i*.1,i) is None


def tunnel_image():
 im=np.full((240,320,3),70,np.uint8);board=np.full((100,100,3),30,np.uint8);stone=(145,172,192)
 cv2.rectangle(board,(0,0),(99,99),stone,4)
 cv2.ellipse(board,(50,54),(37,37),0,180,360,stone,17)
 cv2.rectangle(board,(12,53),(28,85),stone,-1);cv2.rectangle(board,(72,53),(88,85),stone,-1)
 im[35:135,150:250]=board
 return im

def test_tunnel_arch_is_a_preparation_candidate_not_just_a_square():
 im=tunnel_image();assert detect_tunnel(im)['visible']
 cv2.rectangle(im,(150,35),(250,135),(140,170,190),-1);cv2.rectangle(im,(158,43),(242,127),(25,25,25),-1)
 assert not detect_tunnel(im)['visible']


def test_a_small_distant_sign_slows_approach_but_cannot_finish_the_run():
 small=np.full((240,320,3),60,np.uint8);small[40:160,120:280]=cv2.resize(sign('no-entry'),(160,120))
 candidate=detect_stop(small);assert candidate['visible'] and candidate['area']<600
 gate=FinishGate('stop')
 for i in range(5):assert gate.update(small,False,.1*i,i) is None
 assert gate.report['stop']['visible'] and not gate.report['stop']['ready_to_stop']
 for i in range(5,8):event=gate.update(sign('no-entry'),False,.1*i,i)
 assert event['kind']=='stop'
