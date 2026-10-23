# Scripts du robot

`suivi_piste_camera.py` charge ROS, vérifie/démarre les pilotes puis lance un seul essai de suivi. `--check` vérifie les imports sans pilotes ; `--diagnostic` mesure les images et la détection sans publication moteur.

`mission_barriere.py` observe et annote le rectangle bleu sans commander les moteurs. `--run` autorise un essai limité à 5 cm/s avec odométrie, après fermeture observée et ouverture confirmée. `--seconds N` borne un essai d’observation.

Les scripts sont prévus pour le robot 11 : domaine 41, Burger, LDS-01. `URMC_ROBOT_DOMAIN` permet de choisir un autre domaine explicitement. Les pilotes existant dans un autre domaine déclenchent une erreur au lieu d’une deuxième instance. Les fichiers de verrou empêchent les lancements concurrents.

Le reste des fichiers fournit les outils techniques d’inventaire, installation et lancement standard. Aucun mot de passe n’est enregistré. Les journaux sont créés dans session_robot11 sur le robot.
