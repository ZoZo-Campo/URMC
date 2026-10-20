# Simulation et essais réels

## Simulation

Depuis le dépôt compilé :

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=99
export ROS_LOCALHOST_ONLY=1
ros2 launch urmc_mission_01 sim.launch.py dashboard:=true
```

Ouvrir http://127.0.0.1:8765 puis cliquer sur Démarrer. Remplacer `01` par `02` à `05` pour changer de mission ; une seule interface par port. Dans un conteneur, publier le port 8765 uniquement sur le localhost du poste. Pause et Recommencer contrôlent la session. Les perturbations sont disponibles dans la page. Ctrl+C ferme les processus de cette commande.

## Robot : contrôleur principal

Démarrer les pilotes de la base et de la caméra selon le TP. Ne pas lancer une deuxième instance si les pilotes tournent déjà. Charger ROS et le workspace dans chaque terminal, puis utiliser le même domaine que les pilotes. Selon le TP : domaine = 30 + numéro du robot.

Topics attendus : `/image_raw/compressed`, `/scan`, `/odom` ; sortie `/cmd_vel` de type `geometry_msgs/msg/Twist`. Le contrôleur exige des capteurs frais et des secteurs LiDAR suffisamment couverts. Les zéros LiDAR ne sont pas considérés comme de l’espace libre.

```bash
ros2 launch urmc_mission_01 robot.launch.py speed:=0.04
```

Le contrôleur est initialement désarmé. Une fois la zone et les capteurs vérifiés, le service `/urmc/m01/enable` (`std_srvs/srv/SetBool`) permet l’armement ou le désarmement. Ne pas utiliser plusieurs contrôleurs de vitesse simultanément. Vérifier l’arrêt physique et le timeout de la base.

## Mission 1 : prototype caméra seule

Fichiers : `missions/urmc_mission_01/essais_robot/`.

- `essai_suivi_camera.py` : 5 s, maximum 0,03 m/s.
- `suivi_piste_camera.py` : maximum 0,05 m/s, rampe d’accélération, ralentissement avec l’erreur, arrêt au damier ou après 90 s.

**Ces scripts commencent à commander les moteurs lorsque les images deviennent valides.** Ils n’utilisent pas le service d’armement du contrôleur principal. Ils sont réservés aux essais surveillés, sur piste libre ; aucune détection ou manœuvre d’évitement d’obstacle n’est fournie par ce mode.

Après avoir préparé le robot, sur demande de l’opérateur :

```bash
python3 missions/urmc_mission_01/essais_robot/suivi_piste_camera.py
```

Arrêt depuis un autre terminal configuré dans le même domaine :

```bash
ros2 service call /urmc/camera_follow/stop std_srvs/srv/Trigger '{}'
```

Perte de caméra : arrêt ; perte visuelle momentanée : vitesse nulle, avec arrêt après 0,6 s et reprise sur trois images valides si la perte est courte. Une interruption du processus commande zéro dans la sortie normale ; cela ne remplace pas un timeout matériel en cas de crash.

## Vidéo continue

Sur le robot, dans le domaine ROS de la caméra :

```bash
python3 outils/retour_video.py --topic /image_raw/compressed
```

Depuis le poste opérateur, remplacer `ADRESSE_DU_ROBOT` :

```bash
ssh -N -L 8080:127.0.0.1:8080 ubuntu@ADRESSE_DU_ROBOT
```

Ouvrir http://127.0.0.1:8080. Le flux MJPEG est continu ; le compteur affiche les images reçues par le serveur. Cette page ne commande pas les moteurs. Pour une caméra brute, utiliser `--raw --topic /image_raw`.
