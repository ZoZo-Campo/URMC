"""Check the canonical mission folders and stable ROS/robot entry points."""
from pathlib import Path
import xml.etree.ElementTree as ET
ROOT=Path(__file__).resolve().parents[1]

def main():
 folders=sorted(p for p in (ROOT/'Missions').iterdir() if p.is_dir())
 assert len(folders)==5
 assert (ROOT/'Missions/COLCON_IGNORE').is_file()
 for number,folder in enumerate(folders,1):
  name=f'urmc_mission_{number:02d}';package=folder/'code/ros2'/name
  assert (folder/'README.md').is_file() and (folder/'OUVRIR.command').is_file()
  assert (ROOT/'src'/name).resolve()==package.resolve()
  assert ET.parse(package/'package.xml').getroot().findtext('name')==name
 for p in ROOT.rglob('*'):
  if '.git' in p.parts:continue
  if p.is_symlink():assert p.exists() and p.resolve().is_relative_to(ROOT),str(p)
 print('OK : cinq dossiers de mission, packages et liens internes cohérents.')
if __name__=='__main__':main()
