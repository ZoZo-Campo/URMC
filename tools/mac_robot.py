"""Mac operator launcher. All motor commands remain on the robot's ROS nodes."""
import argparse,io,json,os,shlex,socket,subprocess,sys,tarfile,tempfile,threading,time,urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
HOST=os.environ.get('URMC_HOST','192.168.42.211')
SSH=['ssh','-o','ControlMaster=no','-o','ControlPath=none','-o','IPQoS=none','-o','ConnectTimeout=6','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2']
TARGET='ubuntu@'+HOST
REMOTE='urmc_essais_20261020'

def remote(command,tty=False,**kwargs):
    return subprocess.run(SSH+(['-t'] if tty else [])+[TARGET,command],check=True,**kwargs)

def command(script,*args):
    return 'python3 "$HOME/'+REMOTE+'/kit_robot/'+script+'" '+shlex.join(args)

def ensure_tunnel(port):
    try:
        with urllib.request.urlopen('http://127.0.0.1:'+str(port)+'/status',timeout=.6) as response:
            if 'frames' in json.load(response):return
    except Exception:pass
    # SSH checks binding atomically; an occupied port is reported, never terminated.
    subprocess.run(['ssh','-o','ControlMaster=no','-o','ControlPath=none','-o','IPQoS=none','-o','ConnectTimeout=6','-o','ServerAliveInterval=5','-o','ServerAliveCountMax=2','-f','-N','-o','ExitOnForwardFailure=yes','-L',f'127.0.0.1:{port}:127.0.0.1:{port}',TARGET],check=True)

def open_when_ready(port):
    for _ in range(100):
        try:
            with urllib.request.urlopen(f'http://127.0.0.1:{port}/status',timeout=.4):pass
            subprocess.run(['open','-a','Google Chrome',f'http://127.0.0.1:{port}/'],check=True);return
        except Exception:time.sleep(.3)
    print('La video tarde a demarrer : consulter la fenetre de mission.',flush=True)

STOP='''import os,signal
from pathlib import Path
names={'suivi_piste_camera.py','mission_barriere.py','mission_priorite.py','mission_enceinte.py','teleop_robot.py'}
count=0
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  if p.stat().st_uid!=os.getuid():continue
  args=p.joinpath('cmdline').read_bytes().decode().split(chr(0))
  if any(Path(a).name in names for a in args if a):os.kill(int(p.name),signal.SIGINT);count+=1
 except (FileNotFoundError,ProcessLookupError,PermissionError):pass
print('Arret demande a',count,'mission(s).')
'''

def install():
    print('Transfert des sources puis compilation sur le robot. Aucun mouvement.',flush=True)
    with tempfile.TemporaryFile() as stream:
        with tarfile.open(fileobj=stream,mode='w:gz',dereference=True) as archive:
            def clean(info):
                if '__pycache__' in info.name or info.name.endswith(('.pyc','.DS_Store')):return None
                info.mtime=0
                return info
            for folder in ('src','kit_robot'):archive.add(ROOT/folder,arcname=folder,filter=clean)
        stream.seek(0)
        remote('mkdir -p "$HOME/'+REMOTE+'" && tar --no-same-owner -xzf - -C "$HOME/'+REMOTE+'"',stdin=stream)
    build='set -e; source /opt/ros/humble/setup.bash; if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi; cd "$HOME/'+REMOTE+'"; colcon build --symlink-install --event-handlers console_direct+ status- --packages-select urmc_core urmc_mission_01 urmc_mission_02 urmc_mission_03 urmc_mission_04'
    remote('bash -c '+shlex.quote(build),tty=True)

def choose():
    choices={'Suivi de piste - lancer un essai':'follow','Barriere bleue - observer sans rouler':'barrier',
        'Barriere bleue - essai avec roulage':'barrier-run','Video permanente - toutes les missions':'video','STOP - arreter les missions':'stop',
        'Installer / mettre a jour le robot':'install','Voir les simulations':'simulation'}
    labels=', '.join(json.dumps(x) for x in choices)
    script='set picked to choose from list {'+labels+'} with title "URMC - Robot 11" with prompt "Que veux-tu faire ?"\nif picked is false then return ""\nreturn item 1 of picked'
    label=subprocess.check_output(['osascript','-e',script],text=True).strip()
    return choices.get(label)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('action',nargs='?',default='menu',choices=['menu','follow','barrier','barrier-run','video','stop','install','simulation'])
    action=parser.parse_args().action
    if action=='menu':action=choose()
    if action is None:return
    if action=='stop':remote('python3 -c '+shlex.quote(STOP));return
    if action=='install':install();return
    if action=='simulation':subprocess.run([sys.executable,str(ROOT/'tools/visuals.py'),'start'],check=True);return
    # Authenticate before creating a tunnel; no driver or movement from this call.
    remote('true')
    port=8080
    ensure_tunnel(port)
    threading.Thread(target=open_when_ready,args=(port,),daemon=True).start()
    if action=='video':
        remote(command('suivi_piste_camera.py','--prepare-only'),tty=True)
        open_when_ready(port);return
    if action=='follow':remote(command('suivi_piste_camera.py'),tty=True)
    elif action=='barrier':remote(command('mission_barriere.py'),tty=True)
    elif action=='barrier-run':remote(command('mission_barriere.py','--run'),tty=True)

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('\nCommande interrompue. Le controleur distant recoit Ctrl+C.')
    except (RuntimeError,subprocess.CalledProcessError) as exc:raise SystemExit(str(exc))
