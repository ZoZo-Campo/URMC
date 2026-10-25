# Validation et limites

## État actuel

| Élément | Vérifications effectuées | Limites |
|---|---|---|
| Interface et deux profils | Chrome, mobile, identité des vidéos/mesures, réglages distincts, commandes anciennes refusées | IP du robot 9 à confirmer |
| Manuel | Maintien, délai de 250 ms, révocation, conflits de publishers et verrous | Essais matériels de la dernière chaîne à refaire |
| Suivi / barrière | Perception, surfaces, états, fermeture/levée, images distinctes et fraîches ; banc ROS avec vitesses remappées | Éclairage et seuils à vérifier sur place |
| Priorité à droite | Distance/zone/gabarit, option de mouvement, compensation odométrique, attente et traversée sans lignes | Le LiDAR ne distingue pas à coup sûr robot et personne |
| Enceinte / SLAM | Vrai SLAM Toolbox sur scans synthétiques, sauvegarde/rechargement du graphe, pose confirmée, zone bornée, obstacle ajouté après cartographie et sortie avant 60 s | Calibration du gabarit, de la carte, des repères et de la localisation sur les robots |
| Palette | Séquence de simulation | Perception réelle, planification des zones et pilote aimant manquants |

La suite comprend les tests de perception, commande, fraîcheur, sécurité HTTP, séparation des robots, grilles orientées, inflation, zones inconnues, sauvegardes et verrous. Lors du rangement, les **163 tests** passent, puis les pilotes barrière et priorité sont exercés dans ROS avec des topics de vitesse de test. La découverte colcon identifie une seule fois les six packages.

## Scripts utiles

- `python3 -m pytest` : suite unique dans `tests/`.
- `tools/test_barrier_runtime.py` : barrière et suivi avec capteurs ROS synthétiques.
- `tools/test_priority_runtime.py` : cible à droite, traversée sans lignes, coupure caméra et reprise.
- `tools/test_priority_simulation.py` : launch ROS de la mission 3.
- `tools/test_enclosure_slam.py` : workflow du banc SLAM ; refuse un serveur qui ne se déclare pas simulé.
- `tools/integration_ros.py` : lancement des cinq bancs historiques.

Les conteneurs de test isolent les simulations des topics matériels. Les cartes et captures dans `docs/images` sont synthétiques et ne représentent pas le terrain réel.

## Avant un essai physique

Vérifier la connexion, les domaines ROS distincts, les topics et TF des capteurs, la cadence et les horodatages, le rayon réel du montage et les marges. Pour l’enceinte, confirmer la localisation et la zone autorisée, puis placer les repères avant l’entrée, à l’intérieur et au-delà de la seconde ouverture.

Un chemin impossible ou inconnu impose l’arrêt. Les contrôles logiciels ne prouvent pas l’absence de collision dans toute situation physique. Une réussite en simulation ne vaut pas validation matérielle générale.
