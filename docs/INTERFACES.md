# Interfaces ROS 2

## Entrées et sorties du mode robot

| Topic | Type | Missions | Convention |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | Toutes | 0 rad vers l’avant, angles positifs vers la gauche ; distances en mètres. `scan_yaw_offset` permet une correction angulaire, pas une transformation TF complète. |
| `/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 1–3 | JPEG BGR décodé avec OpenCV. Caméra regardant vers l’avant. |
| `/image_raw` | `sensor_msgs/msg/Image` | 1–3 | Alternative via `image_transport:=raw` ; conversion cv_bridge vers BGR. |
| `/odom` | `nav_msgs/msg/Odometry` | 1, 2, 3, 5 | Pose continue dans `odom`, quaternion normalisé. Un saut supérieur à 15 cm entre messages désarme. |
| `/urmc/storage_observations` | `std_msgs/msg/String` JSON | 5 | Contrat expérimental ci-dessous ; pas un détecteur de zones fourni. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Toutes | Avance ≥ 0 ; vitesse limitée à 0,10 m/s et rotation à ±0,7 rad/s. |
| `/urmc/magnet_request` | `std_msgs/msg/Bool` | Toutes | Demande logique seulement. Aucun pilote GPIO/Grove fourni. Ne pas brancher à un actionneur sans intégration dédiée. |
| `/urmc/mXX/status` | `std_msgs/msg/String` JSON | Toutes | État, armement, fraîcheur, vitesse calculée, demande aimant. |

Les missions 1–3 n’utilisent pas de détection par vérité terrain : leurs images sont analysées par OpenCV. La mission 4 ne crée aucun abonnement caméra par défaut ; une assistance explicitement activée hors compétition peut lire le flux compressé. La mission 5 reçoit actuellement des observations de haut niveau dont la perception réelle reste à développer.

Services `/urmc/mXX/enable` (`std_srvs/srv/SetBool`) et `/urmc/mXX/reset` (`std_srvs/srv/Trigger`). `enable=false` commande zéro. Un reset exige désarmement et absence de charge tenue. Une faute de stockage conserve la demande d’aimant ; aucune relâche automatique sur faute n’est imposée ici.

Les remappings sont explicités dans `urmc_core/urmc_core/launching.py`. Le mode simulation utilise exclusivement `/urmc_sim/mXX/*`. Il ne doit publier aucun `/cmd_vel` global.

## Fraîcheur et arrêt

Les messages Image/LaserScan/Odometry doivent avoir un en-tête daté ; timestamp nul, futur ou âgé de plus de 300 ms est refusé aux réglages par défaut. Le nœud vérifie aussi le temps écoulé depuis réception. Les secteurs LiDAR insuffisamment couverts ou invalides sont traités comme inconnus, pas comme libres. `+inf` représente une absence de retour jusqu’à `range_max` ; cette convention doit être compatible avec le pilote utilisé.

Le timer de commande tourne à 20 Hz. La perte d’un capteur requis après fonctionnement désarme le nœud. Le capteur revenu ne réarme pas automatiquement. L’arrêt logiciel ne remplace pas le timeout de commande de la base et l’arrêt physique du robot.

Le modèle suppose le scanner aligné et proche du centre du robot. La largeur utilisée pour le corridor d’évitement est 22 cm ; les tests de collision sont un disque simplifié, pas le volume 3D du Burger et de sa palette.

## JSON stockage

Le fournisseur doit renouveler le message, y compris lorsqu’il n’y a pas de nouvelle cible. `stamp` est le temps ROS en secondes. Ne pas transférer ces valeurs synthétiques sur le vrai robot comme des positions réelles.

```json
{
  "stamp": 0.0,
  "source_id": 1,
  "target_id": 2,
  "pallet": true,
  "pickup_goal": [0.3, 0.0, 0.0],
  "drop_goal": [0.9, 0.0, 0.0],
  "exit_goal": [1.15, 0.0, 0.0],
  "at_pickup": false,
  "grip_confirmed": false,
  "at_drop": false,
  "orientation_ok": false,
  "path_allowed": false,
  "released_confirmed": false,
  "zone_exited": false,
  "forbidden_zone": false
}
```

Les buts `[x, y, yaw]` sont exprimés dans le même repère que `/odom`. Le contrôleur fourni rejoint des poses simples ; il ne calcule pas une route évitant toutes les zones interdites. Le fournisseur de perception/planification devra produire un chemin admissible et les confirmations fiables. Le scénario livré est rectiligne et n’a pas de zone interdite sur sa route.

La libération est conditionnée à `at_drop` et `orientation_ok`, mais le maintien de l’orientation tout au long du transport reste à valider sur un parcours non rectiligne. Une pile autonome complète de stockage nécessitera perception, planification et retour matériel de prise.


## Interface de simulation interactive

Exclusivement sous `/urmc_sim/mXX` : services `sim/start`, `sim/pause`, `sim/restart` de type `std_srvs/srv/Trigger`. `sim_session` transporte le temps de session, l’état pause/marche et un numéro de remise à zéro. Ces services et cet abonnement ne sont pas créés en mode robot.

Le dashboard passe uniquement par ces services et les paramètres du simulateur. Les POST HTTP sont limités aux actions/paramètres autorisés et à la même origine locale. Aucun endpoint HTTP ne publie de vitesse ni ne commande le robot physique. Le mode robot conserve ses abonnements capteurs, son service d’armement et son watchdog.

Par défaut, `sim.launch.py` attend Démarrer. `autostart:=true` est réservé aux essais automatisés. Les tests automatiques restent reproductibles, avec les mêmes fonctions de mission que les essais manuels.

## Mission 1 : récupération et détour (13 octobre 2026)

La mission 1 exige désormais `/odom` frais, y compris pour le suivi simple, afin de pouvoir effectuer son détour local. Les bandes sont analysées dans la caméra ; le contrôleur ne reçoit pas la carte du simulateur. Une bordure seule est estimée pendant au plus 8 secondes à partir de la largeur observée auparavant. Deux bordures absentes déclenchent un balayage sur place, limité à 18 secondes, puis TIMEOUT. Trois images distinctes sont nécessaires pour reprendre après une recherche.

Un obstacle détecté à moins de 42 cm déclenche un détour de 32 cm latéralement puis 72 cm vers l’avant, si un côté paraît assez libre. Les distances sont relatives à la pose mesurée au début de la manœuvre, pas à la carte simulée. Le LiDAR et le garde de commande restent actifs ; un obstacle trop proche arrête le mouvement. Il s’agit d’une manœuvre locale pour un petit obstacle isolé, pas d’un planificateur général. Vérifier les dimensions, les limites du parcours et l’autorisation de quitter la piste avant un essai réel.

## Pilote caméra / LiDAR de la console — mission 3

`kit_robot/mission_priorite.py` utilise les topics absolus `/image_raw/compressed`, `/scan`, `/odom`, `/cmd_vel` ; QoS capteurs BEST_EFFORT, KEEP_LAST profondeur 1. Les en-têtes et délais de réception doivent être récents : caméra <250 ms, LiDAR/odom <300 ms. Les secteurs avant et droit doivent être couverts ; `+inf` signifie libre jusqu’à range_max, les mesures invalides ne signifient pas libre.

`/urmc/priority/status` publie le mode, l’état, la raison, les vitesses, les distances avant/droite, les cibles suivies (`id`, x/y relatifs, gabarit, distance LiDAR–cible, vitesse d’approche, état retenu) et la progression de traversée. `/urmc/priority/stop` et `/urmc/camera_follow/stop` (Trigger) arrêtent le pilote de roulage. L’observation ne publie aucun Twist. Le retour vidéo écoute cet état ; sa carte permanente lit `/scan` en lecture seule.

Le pilote de console se met en pause dès qu’un capteur manque ; après retour de tous les capteurs, la libération de la voie est confirmée à nouveau avant de poursuivre la traversée. Le cap d’origine n’est pas recalé pendant la pause. Un saut d’odométrie, une déviation excessive ou une distance de traversée dépassée terminent l’essai. Ce comportement de reprise est distinct du désarmement du lanceur ROS générique décrit plus haut.

Dans les réglages de console, l’orientation LiDAR est en degrés. Le paramètre ROS du lanceur générique `scan_yaw_offset` reste en radians. Le modèle suppose le LiDAR proche du centre du robot ; il ne remplace pas une transformation TF complète.

## Mission 4 cartographiée — console

Le worker `mission_enceinte.py` lit `/scan`, `/odom`, `/map`, TF `map→base_footprint` et `base_footprint→frame_du_scan`, et `/battery_state` si disponible. En mode sans SLAM, il ignore `/map` et calcule une pose relative à l’odométrie initiale, limitée à un rayon de 3 m. Aucun abonnement caméra par défaut ; l’option d’essai crée un abonnement BEST_EFFORT profondeur 1 à `/image_raw/compressed` puis le détruit à la désactivation. Un publisher `/cmd_vel` n’existe que pendant le roulage supervisé ; STOP publie zéro, le détruit et libère les verrous partagés. Le SLAM peut continuer à fonctionner sans moteur.

Les mutations passent par le jeton privé de l’agent puis localhost:8093. Nom de carte validé, sauvegardes horodatées, graphes et métadonnées séparés ; pas de chemin libre fourni par le navigateur. Les commandes pose/zone/repères/roulage portent l’identifiant de session de carte et sont refusées s’il a changé. La pose doit être confirmée après rechargement.

La planification emploie la carte de localisation et une couche LiDAR actualisée, la zone autorisée, l’empreinte avec marge et les points LiDAR actuels. Les nouveaux impacts bloquent immédiatement ; trois scans distincts libres peuvent libérer un obstacle enregistré, sans changer le fichier de carte. La vue locale sans SLAM est une grille rayonnée en odométrie, sans graphe ni relocalisation. Le mode `task=goal` vise uniquement `route.exit`, choisi au clic et jamais déplacé ; `task=enclosure` conserve les trois étapes. Les cellules inconnues restent bloquées. Le contrôle à 20 Hz vérifie les scans/odom <300 ms, TF récent, conflit de publisher et collision sur un court horizon ; le calcul A* se fait dans un thread séparé et n’immobilise pas ce contrôle. Une perte de capteur met la vitesse à zéro ; sa réception à nouveau permet une reprise tant que la mission et son délai restent valides. Saut de localisation ou délai de 60 s : essai terminé.

Le banc SLAM utilise les mêmes conventions sous `/urmc_sim/m04_slam/*` et des topics TF remappés, jamais un publisher de vitesse matériel. La carte de simulation et ses graphes ne sont pas réutilisables comme carte réelle.

L’assistance caméra (`action=camera`, booléen `enabled`) ne se change qu’à l’arrêt. Elle refuse les images datées de plus de 350 ms, exige une réception de moins de 500 ms et une luminosité suffisante ; un changement de scène à l’arrêt impose une pause de 800 ms. Elle ne déduit ni identité ni distance d’obstacle. La luminosité minimale et la proportion de pixels modifiés sont réglables dans la console. Le mode officiel conserve cette assistance désactivée.
