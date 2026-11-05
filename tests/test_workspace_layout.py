"""Transfers remain standalone after the mission-based source reorganization."""
import sys,tarfile,tempfile
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from operator_console import tempfile_archive

def test_robot_transfer_has_self_contained_mission_links():
 with tempfile_archive() as stream:
  with tarfile.open(fileobj=stream,mode='r:gz') as archive:
   with tempfile.TemporaryDirectory() as folder:
    archive.extractall(folder)
    root=Path(folder)
    for name in ['kit_robot/suivi_piste_camera.py','kit_robot/mission_barriere.py','kit_robot/mission_priorite.py','kit_robot/mission_enceinte.py','kit_robot/nav2_bridge.py','src/urmc_core/urmc_core/barrier.py','src/urmc_mission_04/package.xml']:
     path=root/name
     assert path.is_file() and path.resolve().is_relative_to(root)
    assert 'ROOT=_WORKSPACE' in (root/'kit_robot/mission_enceinte.py').read_text()
