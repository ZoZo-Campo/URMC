# Socle commun ROS 2

- `node.py` : abonnements, fraîcheur, armer/désarmer, publication Twist.
- `control.py` : commandes bornées, secteurs LiDAR, recherche de corridor et primitives de déplacement.
- `vision.py` : HSV et heuristiques de marqueurs, sans modèle appris.
- `sim.py` : cinématique 2D, images et rayons LiDAR synthétiques ; observations de stockage issues de la vérité du simulateur.
- `launching.py` : paramètres et remappings communs aux cinq lanceurs.
- `dashboard.py` et `web/index.html` : visualisation et contrôles réservés au simulateur.

Les décisions de chaque mission sont dans le package `urmc_mission_XX`, fichier `logic.py`. Un seul nœud de mission commande la base à la fois. Voir les guides à la racine du workspace.
