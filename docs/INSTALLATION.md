# Installation ROS 2

Prérequis : Ubuntu 22.04, ROS 2 Humble, `colcon`, `rosdep`, Python 3. Les chemins ci-dessous supposent que le dépôt a été cloné dans `~/URMC`.

```bash
git clone https://github.com/ZoZo-Campo/URMC.git ~/URMC
cd ~/URMC
source /opt/ros/humble/setup.bash
rosdep install --from-paths commun missions --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --base-paths commun missions
source install/setup.bash
```

Les six packages restent nécessaires à l’ensemble des missions. Pour compiler uniquement la mission 1 : ajouter `--packages-select urmc_core urmc_mission_01` à `colcon build`.

Le Mac peut servir de poste de visualisation via SSH. L’exécution ROS sur Mac a été faite dans Ubuntu/Humble ARM64 via Docker, pas avec un ROS macOS natif. Le dépôt est organisé comme un workspace Linux ; les raccourcis privés du poste de développement ne sont pas nécessaires.

Tests unitaires après compilation :

```bash
python3 -m pytest -q tests
```

Ces tests nécessitent pytest en plus des dépendances ROS déclarées. Ils ne commandent pas un robot réel.
