"""Transfers remain standalone after the mission-based source reorganization."""
import sys,tarfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from operator_console import tempfile_archive

def test_robot_transfer_contains_real_files_not_desktop_symlinks():
 with tempfile_archive() as stream:
  with tarfile.open(fileobj=stream,mode='r:gz') as archive:
   names={m.name:m for m in archive.getmembers()}
   for path in ['kit_robot/suivi_piste_camera.py','kit_robot/mission_barriere.py','kit_robot/mission_priorite.py','kit_robot/mission_enceinte.py','src/urmc_core/urmc_core/barrier.py','src/urmc_mission_04/package.xml']:
    assert path in names and names[path].isfile() and not names[path].issym()
   text=archive.extractfile('kit_robot/mission_enceinte.py').read().decode()
   assert 'ROOT=_WORKSPACE' in text
