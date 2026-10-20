# Mission 4 — Enceinte au LiDAR

Les exigences sont décrites dans [REGLES.md](REGLES.md) et [regles.json](regles.json).

- Logique : `urmc_mission_04/logic.py`.
- Paramètres : `config/mission.yaml`.
- Lancements : `launch/sim.launch.py` et `launch/robot.launch.py`.
- Socle partagé : `../../commun/urmc_core`.

Après compilation :

```bash
ros2 launch urmc_mission_04 sim.launch.py dashboard:=true
```

Navigation locale dans une enceinte simple, sans garantie sur une géométrie quelconque. Aucune caméra utilisée.

Voir [les consignes d’utilisation](../../docs/UTILISATION.md) avant un lancement sur le robot.
