# Mission 5 — Stockage de palette

Les exigences sont décrites dans [REGLES.md](REGLES.md) et [regles.json](regles.json).

- Logique : `urmc_mission_05/logic.py`.
- Paramètres : `config/mission.yaml`.
- Lancements : `launch/sim.launch.py` et `launch/robot.launch.py`.
- Socle partagé : `../../commun/urmc_core`.

Après compilation :

```bash
ros2 launch urmc_mission_05 sim.launch.py dashboard:=true
```

La simulation fournit les observations des zones et la confirmation de prise. Perception réelle, planification et pilote matériel restent à développer ; armement réel bloqué par défaut.

Voir [les consignes d’utilisation](../../docs/UTILISATION.md) avant un lancement sur le robot.
