# Utilisation

## Caméra et essais depuis le Mac

Dans le dossier du dépôt :

```bash
python3 tools/mac_robot.py video
```

Ouvrir **http://127.0.0.1:8080/**. Garder cette page ouverte pendant toutes les missions. Les images viennent de `/image_raw/compressed` ; le serveur reste sur le localhost du robot, accessible par tunnel SSH. La page affiche la mission, la raison des pauses, l’âge source côté serveur et le temps de transfert. Une caméra périmée ou un réseau interrompu ne sont pas présentés comme du direct.

Puis lancer un seul essai :

```bash
python3 tools/mac_robot.py follow       # suivi seul, maximum 90 s
python3 tools/mac_robot.py barrier      # observation barrière, aucun moteur
python3 tools/mac_robot.py barrier-run  # suivi + barrière, sans durée maximale
```

Les commandes de roulage démarrent les moteurs lorsque leurs conditions sont remplies. Arrêt : bouton STOP de la vidéo, Ctrl+C dans le terminal de la mission, ou `python3 tools/mac_robot.py stop`. La vidéo reste active.

Suivi : deux bordures jaune/blanche obligatoires, sans côté de couleur imposé. Barycentres jaune/blanc, centre vert et axe cyan. Barrière : panneau bleu horizontal fermé, vertical levé, confirmation stable avant reprise. Un début d’ouverture observé suivi d’une disparition stable peut aussi confirmer la levée dans ce prototype ; une perte de caméra ne compte jamais comme une disparition du panneau. Cette hypothèse doit être vérifiée avec le montage réel.

La barrière fermée, un panneau ambigu, une piste manquante ou des capteurs périmés provoquent une vitesse nulle. Après une erreur de piste, la reprise exige trois images distinctes valides. Les scripts ne font pas d’évitement LiDAR. Le bouton STOP n’est pas un arrêt matériel en cas de crash ; le timeout de la base doit rester configuré.

## Lancement directement sur le robot

```bash
python3 ~/urmc_essais_20261020/kit_robot/suivi_piste_camera.py
python3 ~/urmc_essais_20261020/kit_robot/mission_barriere.py --run
```

Choisir une seule de ces commandes. Pour un essai barrière borné : ajouter `--seconds 30`. Sans cette option, il n’y a pas de limite de durée. La commande Mac `video` prépare le tunnel nécessaire pour regarder la page.

## Simulation sur Mac

```bash
bash tools/mac.sh 01
```

Remplacer `01` par `02`, `03`, `04` ou `05`. Interface : http://127.0.0.1:8765/. Cliquer Démarrer ; Pause et Recommencer contrôlent la session. Ctrl+C termine cette simulation. Pour ouvrir les cinq : `python3 tools/visuals.py start` ; arrêt : `python3 tools/visuals.py stop`. Dans ce mode les ports vont de 8765 à 8769.

## Contrôleurs standard ROS

Après compilation, `ros2 launch urmc_mission_01 sim.launch.py dashboard:=true` lance la simulation. Sur robot : `ros2 launch urmc_mission_01 robot.launch.py speed:=0.04`, avec les pilotes déjà prêts, le même domaine ROS et tous les capteurs requis. Les contrôleurs standard restent désarmés au démarrage ; service `/urmc/m01/enable` (`std_srvs/srv/SetBool`) pour l’armement/désarmement. Remplacer `01` selon la mission.

Le script `kit_robot/lancer.sh` permet les modes observation/roulage pour 01–04, avec le profil matériel renseigné. La mission 05 réelle reste bloquée faute d’intégration matérielle. Les essais caméra du menu et les contrôleurs standard sont deux chemins distincts ; ne pas les exécuter simultanément.
