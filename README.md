# URMC — Robotique mobile autonome

Préparation à la compétition **URMRC 2026 / Robohub**, avec un **TurtleBot3 Burger et ROS 2 Humble**. Les épreuves demandent de suivre une piste, franchir une barrière, gérer une priorité, sortir d’une enceinte et transporter une palette. Les documents officiels fournis à l’équipe sont conservés dans [docs/documents](docs/documents) ; ce dépôt n’est pas une publication des organisateurs.

## Lancer depuis le Mac

Après [installation](docs/INSTALLATION.md), ouvrir `OUVRIR URMC.command`, ou utiliser un Terminal dans le dossier du dépôt :

```bash
python3 tools/mac_robot.py video        # caméra seule, sans roulage
python3 tools/mac_robot.py follow       # un essai de suivi de piste
python3 tools/mac_robot.py barrier-run  # suivi avec arrêt/reprise à la barrière
python3 tools/mac_robot.py stop         # arrêter les missions
```

**Vidéo permanente : http://127.0.0.1:8080/**. Elle reste accessible après l’arrêt d’une mission. Les raccourcis créent le tunnel SSH automatiquement ; le robot doit rester allumé et joignable. Relancer la vidéo après un redémarrage du robot. Le bouton STOP agit sur la mission active.

La procédure courte est dans [JOUR COMPETITION.txt](JOUR%20COMPETITION.txt). Les lancements de roulage commencent à commander les moteurs lorsque leurs conditions sont remplies. Ne pas lancer deux contrôleurs simultanément.

## Missions et état réel

| Package | Objectif | État |
| --- | --- | --- |
| [01 — Suivi de piste](src/urmc_mission_01) | Suivre les deux bordures et s’arrêter au marqueur | Suivi avec virage observé sur robot ; damier et STOP réels restent à valider |
| [02 — Barrière](src/urmc_mission_02) | S’arrêter devant la barrière bleue et reprendre le suivi après ouverture | Enchaînement suivi/arrêt/reprise observé avec moteurs ; robustesse encore en réglage |
| [03 — Priorité](src/urmc_mission_03) | Céder le passage à droite | Prototype simulé ; traversée tout droit seulement |
| [04 — Enceinte LiDAR](src/urmc_mission_04) | Éviter les obstacles et sortir | Prototype réactif testé en simulation simple |
| [05 — Stockage](src/urmc_mission_05) | Prendre, transporter et déposer une palette | Simulation ; perception, préhension et calibration matérielles à intégrer |

Le suivi est parfois appelé **mission 0** dans les fichiers de l’équipe ; son package ROS reste `urmc_mission_01`. Chaque package contient ses sources, paramètres et règles.

Les scripts réellement lancés depuis le menu sont dans `kit_robot/`. Ils sont distincts des contrôleurs standard des packages :

- `suivi_piste_camera.py` : deux bordures jaune/blanche obligatoires, leur côté peut être inversé ; maximum 5 cm/s, un essai limité à 90 s.
- `mission_barriere.py` : observation par défaut ; `--run` suit la piste avant et après la barrière, maximum 5 cm/s, sans limite de durée par défaut. Barrière horizontale = fermée, verticale = levée. Fin au damier ou arrêt demandé. Une vue ambiguë ou une piste perdue provoque une pause ; reprise après confirmation. Les deux bordures restent obligatoires.
- `retour_video.py` : caméra JPEG compressée, annotations et raison des pauses, dernières images demandées successivement pour éviter une longue file vidéo.

Ces scripts expérimentaux n’assurent pas l’évitement des obstacles au LiDAR. Le contrôleur standard exige, lui, des secteurs LiDAR suffisamment couverts. Ses zéros ne sont jamais interprétés comme de l’espace libre. La mission standard 02 conserve son scénario de passage limité ; le script expérimental poursuit le suivi après la barrière.

## Organisation

```text
src/           Les six packages ROS 2 : socle commun et cinq missions
kit_robot/     Scripts réels, pilotes et retour vidéo
tools/         Menu Mac, transfert, simulateur et validations
tests/         Tests de perception, logique et serveur vidéo
docs/          Installation, utilisation, interfaces et documents de compétition
```

## Simulation et validation

Simuler une mission sur le Mac : `bash tools/mac.sh 01` (remplacer `01` par `02` à `05`). Interface : http://127.0.0.1:8765/. Les simulations utilisent Docker/Colima et un modèle 2D simplifié, pas Gazebo ni le parcours officiel.

La validation du 22 octobre comprend **96 tests unitaires** et un scénario ROS du script barrière, avec les vitesses remappées sur un topic de test en réseau Docker isolé. Il vérifie les arrêts, reprises, pertes de capteurs et limites de vitesse. Les essais matériels ont révélé des détections instables ; les derniers correctifs doivent encore être validés sur tout le parcours. Voir [VALIDATION.md](docs/VALIDATION.md).

## Références

- [Tutoriel pédagogique TurtleBot3 line following](https://github.com/loiccu/turtlebot3_line_following).
- Documents de compétition et TP fournis à l’équipe, avec les mentions de leurs auteurs.
- Packages TurtleBot3 de ROBOTIS.

Les PDF restent la propriété de leurs auteurs.
