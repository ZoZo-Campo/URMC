"""Start/stop the five local ROS dashboards; never contact the robot."""
import concurrent.futures, json, os, subprocess, sys, time, urllib.request
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
os.environ['PATH']='/opt/homebrew/bin:/usr/local/bin:'+os.environ.get('PATH','')

def call(*args,check=True):
    return subprocess.run(args,text=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=check)

def stop(i):
    name=f'urmc-visuel-{i:02d}'
    info=call('docker','inspect',name,check=False)
    if info.returncode:return
    data=json.loads(info.stdout)[0]
    if data['Config'].get('Labels',{}).get('urmc.visual')!='true':
        raise RuntimeError('Nom déjà utilisé par un autre conteneur : '+name)
    call('docker','stop','-t','5',name)

def start(i):
    stop(i)
    name=f'urmc-visuel-{i:02d}';port=8764+i
    call('docker','run','-d','--rm','--name',name,'--label','urmc.visual=true',
         '-p',f'127.0.0.1:{port}:8765','-v',f'{ROOT}:/project:ro',
         'urmc-humble:local','bash','/project/tools/container_run.sh',f'{i:02d}')
    url=f'http://127.0.0.1:{port}/'
    for _ in range(120):
        try:
            with urllib.request.urlopen(url+'state',timeout=1) as response:
                state=json.load(response)
            if state.get('status',{}).get('mission')==i:
                call('open','-a','Google Chrome',url+'?multi=1')
                print(f'Mission {i} ouverte : {url}',flush=True)
                return
        except Exception:pass
        time.sleep(.5)
    logs=call('docker','logs','--tail','10',name,check=False)
    raise RuntimeError(f'Mission {i} non disponible : {logs.stdout} {logs.stderr}')

if __name__=='__main__':
    action=sys.argv[1] if len(sys.argv)>1 else 'start'
    if action not in ('start','stop'):raise SystemExit('start ou stop')
    if action=='start' and call('docker','info',check=False).returncode:
        call('colima','start')
    if action=='start' and call('docker','image','inspect','urmc-humble:local',check=False).returncode:
        subprocess.run(['docker','build','-t','urmc-humble:local',str(ROOT)],check=True)
    with concurrent.futures.ThreadPoolExecutor(max_workers=5) as pool:
        futures=[pool.submit(start if action=='start' else stop,i) for i in range(1,6)]
        failures=[]
        for f in futures:
            try:f.result()
            except Exception as exc:failures.append(str(exc))
    if failures:raise SystemExit('\n'.join(failures))
    print('Les cinq simulations sont arrêtées.' if action=='stop' else 'Cinq simulations ouvertes. Pour arrêter : ARRETER_LES_5.command. Relancer VOIR_LES_5.command redémarre les cinq scènes.',flush=True)
