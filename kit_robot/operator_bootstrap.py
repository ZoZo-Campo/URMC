"""Load the robot ROS workspace before importing ROS modules."""
import os,sys
from pathlib import Path

def bootstrap(script):
 script=Path(script).resolve()
 if os.environ.get('URMC_OPERATOR_BOOTSTRAPPED')==str(script):return
 workspace=next((p for p in script.parents if (p/'install/setup.bash').exists()),None)
 if workspace is None:raise SystemExit('Workspace robot non compilé. Utiliser Connecter / préparer dans la console.')
 env=os.environ.copy();env.update(URMC_OPERATOR_BOOTSTRAPPED=str(script),ROS_DOMAIN_ID=env.get('URMC_ROBOT_DOMAIN','41'),ROS_LOCALHOST_ONLY=env.get('URMC_LOCALHOST_ONLY','0'),TURTLEBOT3_MODEL='burger',LDS_MODEL='LDS-01')
 shell='set -e; source /opt/ros/humble/setup.bash; if [[ -f "$HOME/turtlebot3_ws/install/setup.bash" ]]; then source "$HOME/turtlebot3_ws/install/setup.bash"; fi; source "$1"; exec /usr/bin/python3 "$2" "${@:3}"'
 os.execvpe('bash',['bash','-c',shell,'operator',str(workspace/'install/setup.bash'),str(script),*sys.argv[1:]],env)
