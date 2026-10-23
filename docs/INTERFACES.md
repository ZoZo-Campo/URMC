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

Les missions 1–3 n’utilisent pas de détection par vérité terrain : leurs images sont analysées par OpenCV. La mission 4 ne crée aucun abonnement caméra. La mission 5 reçoit actuellement des observations de haut niveau dont la perception réelle reste à développer.

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
