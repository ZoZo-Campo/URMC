# Mission 1 — Suivi de piste et arrêts

Les exigences sont décrites dans [REGLES.md](REGLES.md) et [regles.json](regles.json).

- Logique : `urmc_mission_01/logic.py`.
- Paramètres : `config/mission.yaml`.
- Lancements : `launch/sim.launch.py` et `launch/robot.launch.py`.
- Socle partagé : `../../commun/urmc_core`.

Après compilation :

```bash
ros2 launch urmc_mission_01 sim.launch.py dashboard:=true
```

Le contrôleur principal utilise caméra, LiDAR et odométrie. Le STOP réel n’est pas intégré. Le dossier essais_robot contient un prototype caméra seule distinct, dont le parcours réel avec virage a été confirmé par l’opérateur le 20 octobre 2026 ; arrêt au damier et STOP encore à valider.

Voir [les consignes d’utilisation](../../docs/UTILISATION.md) avant un lancement sur le robot.
