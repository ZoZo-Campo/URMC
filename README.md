# URMC — Robotique mobile autonome

Projet de préparation à la compétition **URMRC 2026 / Robohub**, avec un **TurtleBot3 Burger** et **ROS 2 Humble**. Le robot doit percevoir son environnement, suivre un parcours et accomplir des missions autonomes à partir de sa caméra, de son LiDAR et de son odométrie.

Ce dépôt rassemble le code des missions, un simulateur 2D de travail et la documentation utile à l’équipe. Le simulateur est simplifié : ce n’est ni Gazebo ni le parcours officiel.

## Les cinq missions

| Mission | Objectif | État actuel |
| --- | --- | --- |
| [1 — Suivi de piste](missions/urmc_mission_01) | Suivre les bandes jaune/blanche, franchir les virages et s’arrêter au marqueur | Suivi réel avec virage confirmé par l’opérateur le 20 octobre ; damier et STOP réels à valider |
| [2 — Barrière](missions/urmc_mission_02) | Attendre devant une barrière puis repartir quand le passage est libre | Prototype simulé ; géométrie réelle à vérifier |
| [3 — Priorité](missions/urmc_mission_03) | Détecter l’intersection et céder le passage à droite | Prototype simulé ; traversée tout droit seulement |
| [4 — Enceinte LiDAR](missions/urmc_mission_04) | Entrer, éviter les obstacles et sortir en utilisant le LiDAR | Prototype réactif testé dans une enceinte simple |
| [5 — Stockage](missions/urmc_mission_05) | Prendre, transporter et déposer une palette dans les zones prévues | Scénario simulé ; perception réelle, trajet et pilote d’aimant à intégrer |

Les exigences détaillées, les barèmes et les ambiguïtés identifiées sont conservés dans les fichiers `REGLES.md` et `regles.json` de chaque mission. Les documents de référence sont dans [docs/documents](docs/documents). Les indications du règlement priment sur les simplifications des scénarios.

## Organisation

```text
missions/       Un package ROS 2 par mission
commun/         Socle ROS, perception, protections et simulateur
outils/         Retour vidéo continu de la caméra réelle
docs/           Installation, essais et documents de compétition
tests/          Tests des contrôleurs et de la perception
```

## Démarrage

Sur Ubuntu 22.04 avec ROS 2 Humble : suivre [l’installation](docs/INSTALLATION.md), puis [les simulations et essais robot](docs/UTILISATION.md).

La mission 1 dispose de deux implémentations distinctes : le contrôleur principal avec caméra/LiDAR/odométrie, et un prototype caméra seule utilisé pour les premiers déplacements réels. Ce dernier commande les moteurs dès son lancement : lire sa documentation avant exécution.

## Validation et limites

Les cinq scénarios ROS synthétiques ont atteint leur fin et 59 tests unitaires ont passé lors de la validation du 13 octobre. Ces résultats ne valident pas le matériel ni les scripts caméra seule ajoutés ensuite. Sur le robot, un premier déplacement de 5 s a été confirmé, puis des suivis à 5 cm/s se sont interrompus sur perte de bordures. Le dernier essai avec un virage a poursuivi le suivi pendant environ 42 s avant un arrêt sur perte de bordures. **L’opérateur confirme que le robot a accompli le parcours avec virage.** Cette observation valide ce parcours d’essai ; elle ne valide pas encore la reconnaissance réelle du damier ou du panneau STOP. Le retour vidéo continu fonctionne. Les mesures LiDAR et la robustesse de la perception réelle restent à améliorer.

## Références

- [Tutoriel pédagogique TurtleBot3 line following](https://github.com/loiccu/turtlebot3_line_following).
- Documents de compétition et TP fournis à l’équipe, conservés avec leurs mentions d’auteur.
- Packages TurtleBot3 de ROBOTIS.

Les PDF restent la propriété de leurs auteurs. Ce dépôt n’est pas une publication officielle des organisateurs.
