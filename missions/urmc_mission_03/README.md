# Mission 3 — Priorité à droite

Les exigences sont décrites dans [REGLES.md](REGLES.md) et [regles.json](regles.json).

- Logique : `urmc_mission_03/logic.py`.
- Paramètres : `config/mission.yaml`.
- Lancements : `launch/sim.launch.py` et `launch/robot.launch.py`.
- Socle partagé : `../../commun/urmc_core`.

Après compilation :

```bash
ros2 launch urmc_mission_03 sim.launch.py dashboard:=true
```

La traversée implémentée est tout droit ; aucun virage d’intersection ni suivi multiobjet complet.

Voir [les consignes d’utilisation](../../docs/UTILISATION.md) avant un lancement sur le robot.
