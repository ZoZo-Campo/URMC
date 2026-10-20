"""Local simulation UI. Mutations are restricted to simulation services/parameters."""
import json,threading,queue,time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import CompressedImage,LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory
from .session import validate_changes

class Dashboard(Node):
    def __init__(self):
        super().__init__('dashboard')
        if not self.get_namespace().startswith('/urmc_sim/'):raise ValueError('Dashboard simulation uniquement')
        self.data={};self.jpeg=b'';self.lock=threading.Lock();self.jobs=queue.Queue(maxsize=12)
        self.control_clients={name:self.create_client(Trigger,'sim/'+name) for name in ('start','pause','restart')}
        self.settings=self.create_client(SetParameters,'simulator/set_parameters')
        self.create_timer(.03,self.process_jobs)
        self.create_subscription(String,'status',lambda m:self.update('status',json.loads(m.data)),10)
        self.create_subscription(String,'sim_ground_truth',lambda m:self.update('truth',json.loads(m.data)),10)
        self.create_subscription(CompressedImage,'image_raw/compressed',self.camera,qos_profile_sensor_data)
        self.create_subscription(LaserScan,'scan',self.scan,qos_profile_sensor_data)
        html=(Path(get_package_share_directory('urmc_core'))/'web'/'index.html').read_bytes();owner=self
        class Handler(BaseHTTPRequestHandler):
            def reply(self,code,body,kind='application/json'):
                self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers()
                try:self.wfile.write(body)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_GET(self):
                route=self.path.split('?')[0]
                with owner.lock:
                    if route=='/state':body=json.dumps(owner.data,allow_nan=False).encode();kind='application/json'
                    elif route=='/camera':body=owner.jpeg;kind='image/jpeg'
                    elif route=='/':body=html;kind='text/html; charset=utf-8'
                    else:self.send_error(404);return
                self.reply(200,body,kind)
            def do_POST(self):
                host=self.headers.get('Host','')
                if host.split(':')[0] not in ('127.0.0.1','localhost') or self.headers.get('Origin')!='http://'+host:
                    self.send_error(403);return
                if self.path!='/control' or self.headers.get('Content-Type','').split(';')[0]!='application/json':
                    self.send_error(400);return
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=4096:raise ValueError('Requête invalide')
                    payload=json.loads(self.rfile.read(size))
                    if not isinstance(payload,dict):raise ValueError('Objet attendu')
                    if set(payload)=={'action'} and payload['action'] in owner.control_clients:request=payload
                    elif set(payload)=={'perturbations'}:request={'perturbations':validate_changes(payload['perturbations'])}
                    else:raise ValueError('Commande refusée')
                    job={'request':request,'event':threading.Event(),'deadline':time.monotonic()+5}
                    owner.jobs.put_nowait(job)
                except (ValueError,TypeError,queue.Full) as exc:
                    self.reply(400,json.dumps({'ok':False,'message':str(exc)}).encode());return
                if not job['event'].wait(5):self.reply(503,b'{"ok":false,"message":"ROS ne repond pas"}');return
                self.reply(200,json.dumps(job['result']).encode())
            def log_message(self,*args):pass
        self.http=ThreadingHTTPServer(('0.0.0.0',8765),Handler)
        threading.Thread(target=self.http.serve_forever,daemon=True).start()
        self.get_logger().info('Interface de simulation sur le port 8765 ; commandes strictement locales')
    def process_jobs(self):
        try:job=self.jobs.get_nowait()
        except queue.Empty:return
        if time.monotonic()>job['deadline']:return
        req=job['request'];action=req.get('action')
        client=self.control_clients[action] if action else self.settings
        if not client.service_is_ready():
            job['result']={'ok':False,'message':'Service ROS indisponible'};job['event'].set();return
        if action:message=Trigger.Request()
        else:message=SetParameters.Request(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in req['perturbations'].items()])
        future=client.call_async(message)
        def done(f):
            try:
                result=f.result()
                if action:ok=result.success;note=result.message
                else:ok=all(r.successful for r in result.results);note='Perturbations appliquées' if ok else '; '.join(r.reason for r in result.results)
                job['result']={'ok':ok,'message':note}
            except Exception as exc:job['result']={'ok':False,'message':str(exc)}
            job['event'].set()
        future.add_done_callback(done)
    def update(self,k,v):
        with self.lock:self.data[k]=v
    def camera(self,msg):
        with self.lock:self.jpeg=bytes(msg.data)
    def scan(self,msg):
        import math
        self.update('scan',[[msg.angle_min+i*msg.angle_increment,float(r)] for i,r in enumerate(msg.ranges) if i%4==0 and math.isfinite(r)])
    def destroy_node(self):
        self.http.shutdown();self.http.server_close();super().destroy_node()

def main():
    rclpy.init();node=Dashboard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
