"""Emergency shortcut: console STOP ALL, or known camera/manual scripts over SSH."""
import concurrent.futures,json,re,subprocess,urllib.request
from pathlib import Path
import mac_robot

def main():
 try:
  with urllib.request.urlopen('http://127.0.0.1:8090/',timeout=1) as response:page=response.read().decode()
  token=re.search(r'name="console-token" content="([^"]+)"',page).group(1)
  req=urllib.request.Request('http://127.0.0.1:8090/api/stop-all',data=b'{}',headers={'X-Console-Token':token,'Content-Type':'application/json'})
  with urllib.request.urlopen(req,timeout=5) as response:result=json.load(response)
  if result.get('success'):print(result['message']);return
 except Exception:pass
 hosts={11:'192.168.42.211',9:'192.168.42.209'}
 try:
  import ipaddress
  profiles=json.loads((Path(__file__).resolve().parents[1]/'session_console/robots.json').read_text())
  for robot in hosts:
   if str(robot) in profiles:hosts[robot]=str(ipaddress.IPv4Address(profiles[str(robot)]['host']))
 except (OSError,ValueError,KeyError):pass
 def stop(item):
  import shlex
  robot,host=item
  try:
   result=subprocess.run(mac_robot.SSH+['-o','BatchMode=yes','ubuntu@'+host,'python3 -c '+shlex.quote(mac_robot.STOP)],capture_output=True,text=True,timeout=9)
   return f'Robot {robot} : '+(result.stdout.strip() if result.returncode==0 else 'arrêt non confirmé : SSH inaccessible.')
  except subprocess.SubprocessError:return f'Robot {robot} : arrêt non confirmé : SSH inaccessible.'
 with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
  for message in pool.map(stop,hosts.items()):print(message)
if __name__=='__main__':main()
