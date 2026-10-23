# Mission 3 — Priorité à droite — ROS 2 Humble

Package `urmc_mission_03`, dépend du socle `urmc_core` livré dans le même workspace. Garde le workspace complet lors du transfert : le code associé commun est dans `src/urmc_core/urmc_core/`.

- `launch/sim.launch.py` : capteurs simulés et vrai nœud ROS 2, namespace isolé.
- `launch/robot.launch.py` : capteurs du robot, départ désarmé.
- `config/mission.yaml` : seuils et réglages.
- `urmc_mission_03/main.py` : point d’entrée de cette mission.
- `urmc_mission_03/logic.py` : décisions propres à la mission.
- `regles.json`, `REGLES.md` : exigences résumées ; voir les PDF originaux.

Après compilation et `source install/setup.bash` :

```bash
ros2 launch urmc_mission_03 sim.launch.py
# Ou, uniquement sur le robot préparé :
ros2 launch urmc_mission_03 robot.launch.py
# Puis dans un second terminal configuré, après vérification des capteurs :
ros2 service call /urmc/m03/enable std_srvs/srv/SetBool '{data: true}'
# Arrêt :
ros2 service call /urmc/m03/enable std_srvs/srv/SetBool '{data: false}'
```

Le mode robot publie sur `/cmd_vel` ; le mode simulation reste sous `/urmc_sim/m03/`. Ne lancer qu’une mission à la fois sur le robot et arrêter tout autre émetteur de vitesse.

## État de développement

Le lanceur robot exige `route:=straight` pour permettre l’armement. Aucun virage au carrefour n’est implémenté.

Guide complet et contrat des topics : `../../LIRE_MOI.md` et `../../INTERFACES.md`.
