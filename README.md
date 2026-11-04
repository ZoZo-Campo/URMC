# URMC — TurtleBot3, ROS 2 et préparation de compétition

Ce projet prépare des TurtleBot3 à la compétition URRMC : suivi de piste (0), barrière (1), priorité à droite (2) et enceinte (4). La palette (5) est en pause ; les challenges collectifs (3 et 6) restent à préparer. Les documents de la rencontre URRMC 2026 sont dans [docs/documents](docs/documents).

Une seule interface sur le Mac permet de sélectionner le robot, ouvrir son Terminal SSH, préparer les programmes, piloter manuellement, lancer les missions et consulter les capteurs. Les programmes ROS 2 commandant les moteurs s’exécutent sur le robot.

## Démarrer

Double-cliquer **OUVRIR URMC.command**, puis ouvrir [la console locale](http://127.0.0.1:8090/). Garder sa fenêtre Terminal ouverte. Choisir le robot et **Connecter / préparer** ; aucun mouvement n’est lancé par la connexion. Les moteurs démarrent uniquement avec le manuel ou le lancement d’une mission.

**STOP.command** ou les boutons STOP demandent l’arrêt. Les étapes de compétition sont dans [JOUR COMPETITION.txt](JOUR%20COMPETITION.txt).

| Mission | Dossier et code | État |
|---|---|---|
| 0 · Suivi de piste | [01_suivi_de_piste](Missions/01_suivi_de_piste/) | Pilotes caméra et banc ROS |
| 1 · Barrière bleue | [02_barriere](Missions/02_barriere/) | Suivi, arrêt et reprise ; réglages caméra à vérifier sur place |
| 2 · Priorité à droite | [03_priorite_a_droite](Missions/03_priorite_a_droite/) | LiDAR, traversée sans lignes et maintien de cap ; essais matériels restants |
| 4 · Enceinte / SLAM | [04_enceinte_slam](Missions/04_enceinte_slam/) | SLAM au choix, arrivée au clic et obstacles actualisés ; bancs ROS validés |
| 5 · Palette | [05_stockage](Missions/05_stockage/) | En pause à la demande de l’équipe ; anciens fichiers conservés |

Les numéros affichés sont ceux du barème officiel. Les dossiers/packages techniques `01`, `02`, `03` correspondent aux missions officielles **0**, **1**, **2** ; leurs noms restent stables pour les robots. Les missions collectives **3** et **6** n’ont pas encore de programme dédié.

Chaque dossier de mission contient un README, son raccourci, son code et, lorsqu’ils existent, les liens vers ses tests. Les règles utiles sont intégrées au README. Le code spécifique n’est pas dupliqué.

L’onglet **Missions** résume ce que fait le robot, les points de chaque action et les manques. Le détail des [exigences, points et vérifications restantes](docs/VALIDATION.md) est tenu à jour. L’arrêt au panneau STOP est intégré au suivi réel, avec choix du marqueur par essai. L’enceinte vérifie l’ouverture au LiDAR avant de la franchir. Le détecteur visuel du panneau du tunnel est disponible hors compétition ; son usage pour le point du barème reste à clarifier.

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

## Vitesses et manuel

Pour un TurtleBot3 Burger, [ROBOTIS annonce](https://emanual.robotis.com/docs/en/platform/turtlebot3/features/) **0,22 m/s** et **2,84 rad/s** au maximum. Les limites de notre logiciel sont **0,08 m/s** pour le manuel/suivi/barrière/priorité, **0,15 m/s** pour l’enceinte ; valeurs initiales **0,05** et **0,10 m/s**. Le manuel limite la rotation à **0,4 rad/s**. Ces limites ne sont pas une mesure des deux robots.

Activer le manuel puis maintenir une flèche. Relâcher, quitter la page ou perdre les commandes pendant 250 ms arrête le mouvement. Après un rechargement de page ou un appui expiré, reprendre les commandes puis appuyer de nouveau. Le message sous le bouton indique si les flèches sont prêtes.

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
