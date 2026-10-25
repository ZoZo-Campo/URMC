# Validation et limites

## État actuel

| Élément | Vérifications effectuées | Limites |
|---|---|---|
| Interface et deux profils | Chrome, mobile, identité des vidéos/mesures, réglages distincts, commandes anciennes refusées | IP du robot 9 à confirmer |
| Manuel | Maintien, délai de 250 ms, révocation, conflits de publishers et verrous | Essais matériels de la dernière chaîne à refaire |
| Suivi / barrière | Perception, surfaces, états, fermeture/levée, images distinctes et fraîches ; banc ROS avec vitesses remappées | Éclairage et seuils à vérifier sur place |
| Priorité à droite | Distance/zone/gabarit, option de mouvement, compensation odométrique, attente et traversée sans lignes | Le LiDAR ne distingue pas à coup sûr robot et personne |
| Enceinte / SLAM | Vrai SLAM Toolbox sur scans synthétiques, sauvegarde/rechargement du graphe, pose confirmée, zone bornée, obstacle ajouté après cartographie et sortie avant 60 s | Calibration du gabarit, de la carte, des repères et de la localisation sur les robots |
| Enceinte sans SLAM | Boucle ROS réelle sur capteurs synthétiques, sans TF map ni graphe, arrivée exacte choisie, détour et obstacle retiré de la vue | Dérive odométrique, limite 3 m ; essais matériels restants |
| Arrivée / caméra d’essai | Chrome, sélection exacte au clic, départ bloqué si données périmées ou pose SLAM non confirmée, assistance activée explicitement, affichage mobile | Caméra : qualité et changement à l’arrêt, sans reconnaissance ni profondeur ; désactivée pour la compétition |
| Palette | Séquence de simulation | Perception réelle, planification des zones et pilote aimant manquants |

La suite comprend les tests de perception, commande, fraîcheur, sécurité HTTP, séparation des robots, grilles orientées, inflation, zones inconnues, sauvegardes et verrous. La suite initiale du rangement comporte 163 tests. Après ajout du choix SLAM/local, de l’arrivée au clic et de l’analyse de l’environnement, **177 tests** passent. Les pilotes barrière et priorité sont aussi exercés dans ROS avec des topics de vitesse de test. La découverte colcon identifie une seule fois les six packages.

## Scripts utiles

- `python3 -m pytest` : suite unique dans `tests/`.
- `tools/test_barrier_runtime.py` : barrière et suivi avec capteurs ROS synthétiques.
- `tools/test_priority_runtime.py` : cible à droite, traversée sans lignes, coupure caméra et reprise.
- `tools/test_priority_simulation.py` : launch ROS de la mission 3.
- `tools/test_navigation_modes_runtime.py` : boucle ROS sans SLAM, obstacle ajouté puis retiré et caméra explicite ; utilise uniquement des topics de simulation.
- `tools/test_enclosure_slam.py` : workflow du banc SLAM ; refuse un serveur qui ne se déclare pas simulé.
- `tools/integration_ros.py` : lancement des cinq bancs historiques.

Les conteneurs de test isolent les simulations des topics matériels. Les cartes et captures dans `docs/images` sont synthétiques et ne représentent pas le terrain réel.

## Avant un essai physique

Vérifier la connexion, les domaines ROS distincts, les topics et TF des capteurs, la cadence et les horodatages, le rayon réel du montage et les marges. Pour l’enceinte, confirmer la localisation et la zone autorisée, puis placer les repères avant l’entrée, à l’intérieur et au-delà de la seconde ouverture.

Un chemin impossible ou inconnu impose l’arrêt. Les contrôles logiciels ne prouvent pas l’absence de collision dans toute situation physique. Une réussite en simulation ne vaut pas validation matérielle générale.
