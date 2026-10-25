# URMC — TurtleBot3, ROS 2 et préparation de compétition

Ce projet prépare des TurtleBot3 à cinq épreuves de robotique mobile : suivre une piste, franchir une barrière, appliquer la priorité à droite, traverser une enceinte avec obstacles et transporter une palette. Les documents de la rencontre URRMC 2026 sont dans [docs/documents](docs/documents).

Une seule interface sur le Mac permet de sélectionner le robot, ouvrir son Terminal SSH, préparer les programmes, piloter manuellement, lancer les missions et consulter les capteurs. Les programmes ROS 2 commandant les moteurs s’exécutent sur le robot.

## Démarrer

Double-cliquer **OUVRIR URMC.command**, puis ouvrir [la console locale](http://127.0.0.1:8090/). Garder sa fenêtre Terminal ouverte. Choisir le robot et **Connecter / préparer** ; aucun mouvement n’est lancé par la connexion. Les moteurs démarrent uniquement avec le manuel ou le lancement d’une mission.

**STOP.command** ou les boutons STOP demandent l’arrêt. Les étapes de compétition sont dans [JOUR COMPETITION.txt](JOUR%20COMPETITION.txt).

| Mission | Dossier et code | État |
|---|---|---|
| 1 · Suivi de piste | [01_suivi_de_piste](Missions/01_suivi_de_piste/) | Pilotes caméra et banc ROS ; anciennement mission 0 |
| 2 · Barrière bleue | [02_barriere](Missions/02_barriere/) | Suivi, arrêt et reprise ; réglages caméra à vérifier sur place |
| 3 · Priorité à droite | [03_priorite_a_droite](Missions/03_priorite_a_droite/) | LiDAR, traversée sans lignes et maintien de cap ; essais matériels restants |
| 4 · Enceinte / SLAM | [04_enceinte_slam](Missions/04_enceinte_slam/) | Cartographie, cartes enregistrées et navigation avec obstacles ; banc SLAM validé |
| 5 · Palette | [05_stockage](Missions/05_stockage/) | Simulation ; perception réelle et pilote de l’aimant manquants |

Chaque dossier de mission contient un README, son raccourci, son code et, lorsqu’ils existent, les liens vers ses tests. Les règles utiles sont intégrées au README. Le code spécifique n’est pas dupliqué.

## Où sont les fichiers ?

```text
Missions/                 Une épreuve par dossier
  01_suivi_de_piste/
  02_barriere/
  03_priorite_a_droite/
  04_enceinte_slam/
  05_stockage/
    README.md             Utilisation, état et règles
    OUVRIR.command        Accès à l’interface
    code/                 Package ROS 2, scripts et perception
kit_robot/                Superviseur, vidéo, manuel et points d’entrée stables
tools/console/            Interface web commune
tools/                    Préparation, conteneurs et vérifications
src/urmc_core/            Socle ROS 2 commun
src/urmc_mission_XX        Liens de compatibilité vers les packages des missions
tests/                    Suite de tests unique
docs/                     Console, interfaces ROS et validation
docs/documents/           Les trois PDF de référence
```

Sur le Bureau, `Missions` et `Documents` ouvrent directement les dossiers correspondants de `Code`. `Code` est l’unique copie Git active. Les anciennes copies et références sont archivées séparément ; elles ne sont pas nécessaires au lancement.

## Préparation et matériel

Robot 11 : `192.168.42.211`, domaine ROS proposé 41. Robot 9 : `192.168.42.209` à confirmer, domaine proposé 39. Les domaines des pilotes sont vérifiés lors de la préparation ; ils doivent être distincts. L’accès SSH par clé est nécessaire à la préparation automatique. Aucun mot de passe ni jeton n’est publié.

Pour le SLAM, transférer d’abord les fichiers avec **Connecter / préparer**, puis exécuter sur le robot :

```bash
bash ~/urmc_essais_20261020/kit_robot/installer_mission4.sh
```

Repréparer ensuite le robot et suivre le [guide de la mission 4](Missions/04_enceinte_slam/README.md). Les cartes réelles et réglages locaux sont exclus de Git.

Les simulations fonctionnent sur le Mac avec Docker/Colima. Le banc SLAM utilise `urmc-slam:local`, préparé par `tools/preparer_slam_mac.command`. Les anciennes simulations utilisent `urmc-humble:local`. Le banc est isolé des topics matériels.

## Développement

Sur Ubuntu avec ROS 2 Humble :

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 -m pytest
```

Les anciens chemins `kit_robot/suivi_piste_camera.py`, `mission_barriere.py` et les autres points d’entrée restent disponibles. Les liens sont suivis lors des transferts : le robot reçoit un workspace autonome, sans dépendre du Mac.

- [Interfaces et conventions ROS](docs/INTERFACES.md)
- [Architecture de la console](docs/CONSOLE.md)
- [Validation et limites connues](docs/VALIDATION.md)

Une réussite en simulation ne remplace pas les essais physiques : vérifier les capteurs, le gabarit, les seuils, les marges et la localisation sur le terrain réel.
