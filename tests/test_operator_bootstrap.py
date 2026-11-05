"""An inherited ROS environment is reused, with the child's robot domain."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'kit_robot'))
import operator_bootstrap

def test_child_reuses_workspace_and_sets_its_domain(tmp_path,monkeypatch):
 install=tmp_path/'install';install.mkdir();(install/'setup.bash').write_text('')
 scripts=tmp_path/'kit_robot';scripts.mkdir();script=scripts/'worker.py';script.write_text('')
 env={'URMC_ROS_WORKSPACE':str(tmp_path),'URMC_ROBOT_DOMAIN':'39','URMC_LOCALHOST_ONLY':'1'}
 monkeypatch.setattr(operator_bootstrap.os,'environ',env)
 calls=[];monkeypatch.setattr(operator_bootstrap.os,'execvpe',lambda *args:calls.append(args))
 operator_bootstrap.bootstrap(script)
 assert calls==[] and env['ROS_DOMAIN_ID']=='39' and env['ROS_LOCALHOST_ONLY']=='1'
 assert env['URMC_OPERATOR_BOOTSTRAPPED']==str(script)

def test_first_start_sources_only_the_complete_workspace(tmp_path,monkeypatch):
 install=tmp_path/'install';install.mkdir();(install/'setup.bash').write_text('')
 scripts=tmp_path/'kit_robot';scripts.mkdir();script=scripts/'worker.py';script.write_text('')
 monkeypatch.setattr(operator_bootstrap.os,'environ',{'URMC_ROBOT_DOMAIN':'41'})
 calls=[];monkeypatch.setattr(operator_bootstrap.os,'execvpe',lambda *args:calls.append(args))
 operator_bootstrap.bootstrap(script)
 assert len(calls)==1
 _,args,env=calls[0]
 assert args[2].count('source ')==1 and env['URMC_ROS_WORKSPACE']==str(tmp_path)
