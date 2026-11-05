"""A camera/map poll and a mapping action must share one ROS worker."""
import sys,threading
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
from mapping_manager import MappingManager

def test_concurrent_requests_share_one_startup(tmp_path,monkeypatch):
    (tmp_path/'session_robot11').mkdir()
    manager=MappingManager(tmp_path)
    entered=threading.Event();release=threading.Event();ready=threading.Event()
    starts=[];errors=[]
    class Process:
        def poll(self):return None
    def spawn(*args,**kwargs):
        starts.append(args);entered.set()
        assert release.wait(2)
        ready.set();return Process()
    def request(*args,**kwargs):
        if not ready.is_set():raise OSError('starting')
        return {'available':True}
    monkeypatch.setattr('mapping_manager.subprocess.Popen',spawn)
    monkeypatch.setattr(manager,'request',request)
    def ensure():
        try:manager.ensure()
        except Exception as exc:errors.append(exc)
    first=threading.Thread(target=ensure);second=threading.Thread(target=ensure)
    first.start();assert entered.wait(2)
    second.start();release.set()
    first.join(2);second.join(2)
    assert not first.is_alive() and not second.is_alive()
    assert not errors
    assert len(starts)==1
