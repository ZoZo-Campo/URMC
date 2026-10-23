"""Afficheur video ROS 2 lecture seule + bouton d'arrete du suivi camera.

Flux MJPEG continu depuis un topic CompressedImage. Le bouton STOP de la page
appelle le service /urmc/camera_follow/stop (std_srvs/srv/Trigger), c'est-a-dire
l'arrete du prototype de roulage camerise. Ce serveur n'ouvre jamais les moteurs
lui-meme : il relaie uniquement une demande d'arrete. Bind localhost ; acces par tunnel SSH.
"""
import argparse,json,threading,time,socket
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from std_srvs.srv import Trigger,SetBool
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy
LATEST=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,history=HistoryPolicy.KEEP_LAST)
from sensor_msgs.msg import CompressedImage,Image

PAGE='''<!doctype html><html lang="fr"><meta charset="utf-8"><title>URMC — direct robot 11</title>
<style>body{background:#14251f;color:white;font:18px system-ui;text-align:center;margin:20px}img{width:960px;max-width:96%;max-height:72vh;object-fit:contain}button{padding:16px;border:0;border-radius:8px;cursor:pointer}#stop{background:#c0392b;color:white;font-size:22px}p{min-height:24px}</style>
<h1>Robot 11 — caméra en direct</h1><p id="mission">Caméra permanente — aucune mission nécessaire</p><p id="state">Connexion…</p><img id="image" alt="Dernière image du robot"><p><button id="stop" onclick="stopMission()">STOP mission</button></p><p id="stopmsg"></p>
<p>Suivi : points jaune et blanc = barycentres ; point vert = centre de piste ; ligne cyan = axe caméra.<br>Barrière : image annotée à gauche, masque bleu à droite.</p>
<script>
const image=document.querySelector('#image'),state=document.querySelector('#state');
let previousURL=null,previousSeq=null,lastNew=performance.now();
async function nextFrame(){
 const started=performance.now();
 try{
  const r=await fetch('/frame.jpg?t='+Date.now(),{cache:'no-store',signal:AbortSignal.timeout(1200)});
  if(!r.ok)throw Error('Caméra indisponible ou image périmée');
  const blob=await r.blob(),url=URL.createObjectURL(blob),seq=r.headers.get('X-Frame-Sequence');
  image.src=url;await image.decode();if(previousURL)URL.revokeObjectURL(previousURL);previousURL=url;
  if(seq!==previousSeq){lastNew=performance.now();previousSeq=seq;}
  const transfer=performance.now()-started,age=Number(r.headers.get('X-Source-Age'));
  const fresh=performance.now()-lastNew<1000;image.style.opacity=fresh?1:.25;
  document.querySelector('#mission').textContent=r.headers.get('X-Mission')||'Caméra permanente';
  state.textContent=fresh?'DIRECT · image '+seq+' · âge caméra au serveur '+age.toFixed(2)+' s · transfert '+Math.round(transfer)+' ms':'IMAGE FIGÉE';
 }catch(e){state.textContent='FLUX INTERROMPU — '+e.message;image.style.opacity=.2;}
 setTimeout(nextFrame,Math.max(0,90-(performance.now()-started)));
}
nextFrame();
async function stopMission(){const b=document.querySelector('#stop'),m=document.querySelector('#stopmsg');b.disabled=true;try{const r=await fetch('/stop',{method:'POST',signal:AbortSignal.timeout(4000)});const j=await r.json();m.textContent=j.message;}catch(e){m.textContent='Arrêt non confirmé : '+e.message;}finally{b.disabled=false;}}
</script></html>'''.encode('utf-8')


class Viewer(Node):
    def __init__(self,topic,raw,stop_service,overlay="none"):
        super().__init__('urmc_video_readonly')
        self.lock=threading.Lock();self.frame=None;self.received=0;self.frames=0
        self.lane_tracker=None;self.overlay=overlay;self.source_stamp=None;self.last_processed=0.
        self.stop_service=stop_service;self.stop_lock=threading.Lock();self.stop_result=None
        self.create_subscription(Image if raw else CompressedImage,topic,self.raw if raw else self.compressed,LATEST)
        self.stop_client=self.create_client(Trigger,stop_service)
        self.missions={};self.mission_label='Camera permanente - aucune mission active'
        self.mission_stop_clients={}
        if overlay=='auto':
            topics={'follow':'/urmc/camera_follow/status','barrier':'/urmc/barrier/status'}
            topics.update({str(i):f'/urmc/m{i:02d}/status' for i in range(1,6)})
            for key,topic_name in topics.items():
                self.create_subscription(String,topic_name,lambda msg,k=key:self.mission_status(k,msg),1)
                if key in ('follow','barrier'):
                    service='/urmc/camera_follow/stop' if key=='follow' else '/urmc/barrier/stop'
                    self.mission_stop_clients[key]=(self.create_client(Trigger,service),Trigger.Request())
                else:
                    req=SetBool.Request();req.data=False
                    self.mission_stop_clients[key]=(self.create_client(SetBool,f'/urmc/m{int(key):02d}/enable'),req)
    def mission_status(self,key,msg):
        try:data=json.loads(msg.data)
        except (ValueError,TypeError):return
        self.missions[key]=(time.monotonic(),data)
    def active_mission(self):
        now=time.monotonic()
        active=[(key,t,data) for key,(t,data) in self.missions.items() if now-t<2.5 and data.get('state') not in ('STOP','DONE')]
        # A camera-only observer must not hide an actual running controller.
        active.sort(key=lambda item:(item[2].get('mode')!='observation',item[1]),reverse=True)
        return active[0] if active else None
    def view_mode(self):
        if self.overlay!='auto':return self.overlay
        active=self.active_mission()
        if not active:
            self.mission_label='Camera permanente - aucune mission active';return 'lane'
        key,_,data=active
        names={'follow':'Suivi de piste','barrier':'Barriere bleue','1':'Suivi de piste','2':'Barriere bleue','3':'Priorite','4':'Enceinte LiDAR','5':'Palette'}
        state=str(data.get('state',''))
        labels={'WAIT_OPEN':'attente ouverture','WAIT_BARRIER_VIEW':'barriere mal visible','WAIT_LANE':'bordure manquante','WAIT_LANE_OFFSET':'piste trop decalee','CONFIRM_LANE':'confirmation piste','WAIT_CAMERA':'attente camera','WAIT_ODOM':'attente odometrie','FOLLOW':'suivi','FOLLOW_AFTER_BARRIER':'suivi apres barriere'}
        self.mission_label=names[key]+' - '+labels.get(state,state)
        if state in ('WAIT_LANE','WAIT_LANE_OFFSET','CONFIRM_LANE'):self.mission_label+=' - '+str(data.get('lane_reason',''))
        return 'barrier' if key in ('2','barrier') else 'lane' if key in ('follow','1','3') else 'none'
    def save(self,data):
        with self.lock:self.frame=data;self.received=time.monotonic();self.frames+=1
    def compressed(self,msg):
        now=time.monotonic()
        if now-self.last_processed<.075:return
        self.last_processed=now
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=self.get_clock().now().nanoseconds/1e9-stamp
        if not 0<=age<.5:return
        self.source_stamp=stamp
        data=bytes(msg.data)
        mode=self.view_mode()
        if mode=='barrier':
            from urmc_core.barrier import detect,annotate
            import cv2,numpy as np
            image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
            if image is not None:self.encode(annotate(image,detect(image),False))
        elif mode=='lane':
            from suivi_piste_camera import lane_details,LaneTracker
            import cv2,numpy as np
            if self.lane_tracker is None:self.lane_tracker=LaneTracker()
            details=lane_details(data,self.lane_tracker)
            image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
            if image is None:return
            image=cv2.resize(image,(320,240));cv2.line(image,(160,90),(160,239),(255,255,0),1)
            if 'roi' in details:
                top,bottom=details['roi'];cv2.rectangle(image,(0,top),(319,bottom),(180,180,180),1)
                for key,color,label in [('yellow_center',(0,255,255),'J'),('white_center',(255,255,255),'B')]:
                    if details.get(key) is None:continue
                    point=tuple(round(v) for v in details[key])
                    cv2.circle(image,point,5,color,2);cv2.putText(image,label,(point[0]+6,point[1]),0,.4,color,1)
            if details['error'] is not None:
                points=[tuple(round(v) for v in details[k]) for k in ('yellow_center','white_center') if details.get(k) is not None]
                target=tuple(round(v) for v in details['target_center'])
                if len(points)==2:cv2.line(image,points[0],points[1],(0,255,0),1)
                cv2.circle(image,target,5,(0,255,0),2)
                cv2.arrowedLine(image,(160,239),target,(0,255,0),2)
                label='erreur %.2f | seuil blanc %s'%(details['error'],details['white_v'])
            else:label=details['reason']
            cv2.rectangle(image,(0,0),(319,24),(0,0,0),-1)
            cv2.putText(image,label,(4,17),0,.4,(0,255,0),1)
            self.encode(image)
        elif data.startswith(b'\xff\xd8'):self.save(data)
        else:
            import cv2,numpy as np
            image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
            if image is not None:self.encode(image)
    def encode(self,image):
        import cv2
        ok,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,55])
        if ok:self.save(data.tobytes())
    def raw(self,msg):
        from cv_bridge import CvBridge
        try:self.encode(CvBridge().imgmsg_to_cv2(msg,'bgr8'))
        except Exception as exc:self.get_logger().warning(str(exc),throttle_duration_sec=5.)
    def request_stop(self):
        request=Trigger.Request()
        if self.overlay=='auto':
            active=self.active_mission()
            if not active:return dict(success=False,message='Aucune mission active detectee. Camera toujours disponible.')
            self.stop_client,request=self.mission_stop_clients[active[0]]
        if not self.stop_client.service_is_ready():
            return dict(success=False,message="Service d'arrete indisponible : le suivi camera ne tourne probablement pas.")
        with self.stop_lock:
            if self.stop_result is None:
                self.stop_result={'event':threading.Event()}
                try:self.stop_client.call_async(request).add_done_callback(self.on_stop)
                except Exception as exc:
                    self.stop_result=None
                    return dict(success=False,message='Echec de l\'appel : '+str(exc))
            pending=self.stop_result
        if not pending['event'].wait(3.0):
            with self.stop_lock:self.stop_result=None
            return dict(success=False,message='Pas de reponse du service dans le delai de 3 s.')
        with self.stop_lock:result=self.stop_result;self.stop_result=None
        status=getattr(result.get('result'),'success',False)
        message=getattr(result.get('result'),'message','') or ('Arrete demande' if status else 'Echec de l\'arrete')
        self.get_logger().info('Demande d\'arrete suivi camera : '+str(status)+' - '+str(message))
        return dict(success=bool(status),message=str(message))
    def on_stop(self,future):
        with self.stop_lock:
            if self.stop_result is None:return
            try:self.stop_result['result']=future.result()
            except Exception as exc:self.stop_result['result']=None;self.stop_result['error']=str(exc)
            self.stop_result['event'].set()


def make_handler(node,page,fps=10.):
    class Handler(BaseHTTPRequestHandler):
        def stream(self):
            self.connection.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,16384)
            self.connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            self.connection.settimeout(.8)
            self.send_response(200)
            self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control','no-store');self.end_headers()
            previous=-1
            try:
                while rclpy.ok():
                    with node.lock:
                        frame=node.frame;sequence=node.frames;age=time.monotonic()-node.received
                    if frame is not None and sequence!=previous and age<2:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+b'\r\n\r\n'+frame+b'\r\n')
                        self.wfile.flush();previous=sequence
                    time.sleep(1./fps)
            except (BrokenPipeError,ConnectionResetError,TimeoutError,OSError):pass
        def respond(self,body,mime,status=200,headers=None):
            self.send_response(status);self.send_header('Content-Type',mime)
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)))
            for key,value in (headers or {}).items():self.send_header(key,str(value))
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_POST(self):
            if self.path.split('?')[0]=='/stop':
                return self.respond(json.dumps(node.request_stop()).encode(),'application/json',200)
            return self.respond(b'Methode inconnue','text/plain',404)
        def do_GET(self):
            with node.lock:frame=node.frame;age=time.monotonic()-node.received;sequence=node.frames;stamp=node.source_stamp
            source_age=node.get_clock().now().nanoseconds/1e9-stamp if stamp is not None else age
            fresh=frame is not None and age<.5 and 0<=source_age<.7
            path=self.path.split('?')[0]
            if path=='/stream.mjpg':return self.stream()
            if path=='/':body=page;mime='text/html; charset=utf-8';status=200
            elif path=='/status':body=json.dumps(dict(fresh=fresh,age=age if frame else None,source_age=source_age,frames=sequence)).encode();mime='application/json';status=200
            elif path=='/frame.jpg' and fresh:return self.respond(frame,'image/jpeg',headers={'X-Frame-Sequence':sequence,'X-Source-Age':round(source_age,3),'X-Mission':node.mission_label})
            else:body=b'Image indisponible';mime='text/plain';status=503
            self.respond(body,mime,status)
        def log_message(self,*args):pass
    return Handler


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--topic',default='/image_raw/compressed');parser.add_argument('--raw',action='store_true');parser.add_argument('--port',type=int,default=8080);parser.add_argument('--stop-service',default='/urmc/camera_follow/stop')
    parser.add_argument('--fps',type=float,default=10.);parser.add_argument('--overlay',choices=['none','lane','auto'],default='none')
    args=parser.parse_args()
    if not 1<=args.fps<=15:parser.error('FPS doit etre entre 1 et 15')
    rclpy.init();node=Viewer(args.topic,args.raw,args.stop_service,args.overlay)
    server=ThreadingHTTPServer(('127.0.0.1',args.port),make_handler(node,PAGE,args.fps))
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print(f'Video locale : http://127.0.0.1:{args.port} ; topic {args.topic} ; stop {args.stop_service}',flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:server.shutdown();server.server_close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
