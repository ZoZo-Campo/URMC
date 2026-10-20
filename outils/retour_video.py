"""Read-only ROS 2 camera viewer. Bind localhost; access through an SSH tunnel."""
import argparse,json,threading,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage,Image

PAGE='''<!doctype html><html lang="fr"><meta charset="utf-8"><title>Video en direct - TurtleBot 11</title><style>body{background:#14251f;color:#fff;font:18px system-ui;text-align:center;margin:30px}img{width:960px;max-width:95%;max-height:75vh;object-fit:contain}p{color:#cbded5}button{padding:12px;border:0;border-radius:8px}</style><h1>TurtleBot 11 - VIDEO EN DIRECT</h1><p id="state">Connexion...</p><img id="image" src="/stream.mjpg" alt="Flux video continu du robot"><p><button onclick="document.querySelector('#image').src='/stream.mjpg?t='+Date.now()">Reconnecter la video</button></p><p>Affichage uniquement - aucune commande moteur.</p><script>let last=0,lastTime=performance.now(),pending=false;setInterval(async()=>{if(pending)return;pending=true;try{const s=await(await fetch('/status',{signal:AbortSignal.timeout(3000)})).json();const now=performance.now(),fps=(s.frames-last)*1000/(now-lastTime);last=s.frames;lastTime=now;document.querySelector('#state').textContent=s.fresh?'EN DIRECT · '+fps.toFixed(0)+' images/s recues · image '+s.frames+' · age '+s.age.toFixed(2)+' s':'Camera interrompue - aucune image recente';document.querySelector('#image').style.opacity=s.fresh?1:.2}catch(e){document.querySelector('#state').textContent='Connexion interrompue - reconnecter la video';document.querySelector('#image').style.opacity=.2}finally{pending=false}},1000)</script></html>'''.encode('utf-8')


class Viewer(Node):
    def __init__(self,topic,raw):
        super().__init__('urmc_video_readonly')
        self.lock=threading.Lock();self.frame=None;self.received=0;self.frames=0
        self.create_subscription(Image if raw else CompressedImage,topic,self.raw if raw else self.compressed,qos_profile_sensor_data)
    def save(self,data):
        with self.lock:self.frame=data;self.received=time.monotonic();self.frames+=1
    def compressed(self,msg):
        # JPEG can be served without decoding; other compressed formats are converted.
        data=bytes(msg.data)
        if data.startswith(b'\xff\xd8'):self.save(data)
        else:
            import cv2,numpy as np
            image=cv2.imdecode(np.frombuffer(data,np.uint8),cv2.IMREAD_COLOR)
            if image is not None:self.encode(image)
    def encode(self,image):
        import cv2
        ok,data=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,75])
        if ok:self.save(data.tobytes())
    def raw(self,msg):
        from cv_bridge import CvBridge
        try:self.encode(CvBridge().imgmsg_to_cv2(msg,'bgr8'))
        except Exception as exc:self.get_logger().warning(str(exc),throttle_duration_sec=5.)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--topic',default='/image_raw/compressed');parser.add_argument('--raw',action='store_true');parser.add_argument('--port',type=int,default=8080)
    args=parser.parse_args();rclpy.init();node=Viewer(args.topic,args.raw)
    class Handler(BaseHTTPRequestHandler):
        def stream(self):
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
                    time.sleep(.0667)
            except (BrokenPipeError,ConnectionResetError,TimeoutError):pass
        def do_GET(self):
            with node.lock:frame=node.frame;age=time.monotonic()-node.received
            path=self.path.split('?')[0]
            if path=='/stream.mjpg':return self.stream()
            if path=='/':body=PAGE;mime='text/html; charset=utf-8';status=200
            elif path=='/status':body=json.dumps(dict(fresh=frame is not None and age<2,age=age if frame else None,frames=node.frames)).encode();mime='application/json';status=200
            elif path=='/frame.jpg' and frame and age<2:body=frame;mime='image/jpeg';status=200
            else:body=b'Image indisponible';mime='text/plain';status=503
            self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def log_message(self,*args):pass
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print(f'Video locale : http://127.0.0.1:{args.port} ; topic {args.topic}',flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:server.shutdown();server.server_close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
