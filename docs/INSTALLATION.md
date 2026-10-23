# Installation

## Poste Mac

Cloner ce dépôt dans `~/URMC` (ou adapter les chemins du guide court) :

```bash
git clone https://github.com/ZoZo-Campo/URMC.git ~/URMC
cd ~/URMC
python3 tools/mac_robot.py menu
```

Le pilotage depuis le Mac utilise Python 3 et SSH ; ROS reste sur le robot. Les simulations nécessitent Docker, éventuellement Colima, et l’image Ubuntu/ROS construite par `tools/mac.sh`. Le script propose les simulations si ces outils sont déjà installés.

Par défaut le menu utilise `ubuntu@192.168.42.211`, domaine ROS 41, workspace robot `~/urmc_essais_20261020`. Pour une autre adresse : `export URMC_HOST=ADRESSE_DU_ROBOT` avant de lancer le menu. Aucun mot de passe n’est enregistré dans le dépôt.

Sur un robot déjà équipé de ROS et des pilotes TurtleBot3, transférer et compiler les missions 01/02 avec :

```bash
python3 tools/mac_robot.py install
```

Cette commande met à jour les sources du workspace robot. Elle ne configure pas un système vierge et ne lance pas le roulage. La clé SSH et l’accès à la machine doivent être préparés avant.

## Workspace Ubuntu 22.04 / ROS 2 Humble

```bash
cd ~/URMC
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/setup.bash
python3 -m pytest -q -p no:cacheprovider tests
```

Les tests nécessitent pytest, OpenCV et NumPy. Le robot nécessite également les pilotes TurtleBot3, la caméra v4l2 et les interfaces `/image_raw/compressed`, `/odom`, `/scan`, `/cmd_vel`. Les scripts autonomes chargent ROS et le workspace puis préparent les pilotes ; le contrôleur standard utilise le profil de `kit_robot/profil_robot.sh`, à renseigner après inventaire.
