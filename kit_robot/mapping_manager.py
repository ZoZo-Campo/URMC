"""Robot-agent gateway to the dedicated ROS SLAM/mission worker."""
import json,os,subprocess,sys,time,urllib.request
from pathlib import Path
class MappingManager:
    def __init__(self,root):self.root=Path(root);self.process=None
    def request(self,path,data=None,timeout=4):
        token=(self.root/'session_robot11/.console_token').read_text().strip()
        req=urllib.request.Request('http://127.0.0.1:8093'+path,data=json.dumps(data).encode() if data is not None else None,headers={'Authorization':'Bearer '+token,'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(req,timeout=timeout) as r:return json.load(r)
        except urllib.error.HTTPError as e:
            try:message=json.load(e)['error']
            except Exception:message='Erreur SLAM'
            raise ValueError(message) from None
    def ensure(self):
        try:self.request('/state',timeout=.3);return
        except Exception:pass
        if self.process is None or self.process.poll() is not None:
            with (self.root/'session_robot11/cartographie.log').open('a') as log:
                self.process=subprocess.Popen([sys.executable,str(self.root/'kit_robot/mission_enceinte.py')],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,env=os.environ.copy(),start_new_session=True)
        for _ in range(30):
            try:self.request('/state',timeout=.3);return
            except Exception:time.sleep(.1)
        raise ValueError('Module cartographie indisponible. Voir cartographie.log et vérifier les dépendances ROS.')
    def summary(self):
        try:return self.request('/state',timeout=.3)
        except Exception:return dict(available=False,active=False,message='Ouvrir Cartographie pour démarrer le module.')
    def stop(self,required=False):
        try:return self.request('/action',{'action':'stop'},timeout=3)
        except Exception as exc:
            if required or (self.process is not None and self.process.poll() is None):raise ValueError('Arrêt enceinte non confirmé : '+str(exc)) from None
            return {'success':True}
