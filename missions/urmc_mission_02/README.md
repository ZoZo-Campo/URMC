# Mission 2 — Barrière

Les exigences sont décrites dans [REGLES.md](REGLES.md) et [regles.json](regles.json).

- Logique : `urmc_mission_02/logic.py`.
- Paramètres : `config/mission.yaml`.
- Lancements : `launch/sim.launch.py` et `launch/robot.launch.py`.
- Socle partagé : `../../commun/urmc_core`.

Après compilation :

```bash
ros2 launch urmc_mission_02 sim.launch.py dashboard:=true
```

La barrière doit être visible à la caméra et au LiDAR pour la logique de reprise actuelle.

Voir [les consignes d’utilisation](../../docs/UTILISATION.md) avant un lancement sur le robot.
