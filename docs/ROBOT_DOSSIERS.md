# Organisation du robot

Le dossier actif est `~/urmc/`. L’ancien chemin `~/urmc_essais_20261020` reste un lien compatible sur le robot 11.

| Accès simple | Mission officielle | Sources |
|---|---|---|
| `mission_0_suivi_ligne/` | 0 : piste, damier et STOP | `Missions/01_suivi_de_piste/` |
| `mission_1_barriere/` | 1 : barrière | `Missions/02_barriere/` |
| `mission_2_priorite_droite/` | 2 : priorité à droite | `Missions/03_priorite_a_droite/` |
| `mission_4_enceinte_slam/` | 4 : enceinte et SLAM | `Missions/04_enceinte_slam/` |

Les accès simples sont des liens : une seule copie du code. Chaque mission regroupe `code/robot`, `code/perception`, `code/ros2`, son guide et ses tests lorsqu’ils existent. Les numéros historiques des paquets ROS sont conservés. La mission collective 3 n’a pas encore de programme combiné dédié ; la mission 5 reste en attente.

- `kit_robot/` : superviseur, vidéo et points d’entrée compatibles.
- `src/` : sources communes et liens vers les paquets des missions.
- `build/`, `install/`, `log/` : compilation ROS 2.
- `cartes/` : cartes enregistrées.
- `session_robot11/` : réglages, jetons locaux et journaux, à ne pas publier.
- `archives/` : sauvegardes, exclues de la compilation.

Installer SLAM : `bash ~/urmc/kit_robot/installer_mission4.sh`. Puis ouvrir Enceinte / SLAM dans la console du Mac et créer une carte. Créer une carte ne lance pas la navigation.

Suivi seul avec moteurs : `python3 ~/urmc/kit_robot/mission_barriere.py --run --ignore-barrier`.

Pour compiler : `cd ~/urmc` puis, après chargement de ROS, `colcon build --base-paths src --symlink-install --packages-select urmc_core urmc_mission_01 urmc_mission_02 urmc_mission_03 urmc_mission_04`.
