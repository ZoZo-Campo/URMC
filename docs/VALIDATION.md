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
| Palette (5), en pause | Ancienne séquence de simulation conservée | Perception réelle, planification des zones et pilote aimant manquants |

La suite comprend les tests de perception, commande, fraîcheur, sécurité HTTP, séparation des robots, grilles orientées, inflation, zones inconnues, sauvegardes et verrous. La suite initiale du rangement comporte 163 tests. Après ajout du choix SLAM/local, de l’arrivée au clic et de l’analyse de l’environnement, **177 tests** passent. Les pilotes barrière et priorité sont aussi exercés dans ROS avec des topics de vitesse de test. La découverte colcon identifie une seule fois les six packages.

## Scripts utiles

- `python3 -m pytest` : suite unique dans `tests/`.
- `tools/test_barrier_runtime.py` : barrière et suivi avec capteurs ROS synthétiques.
- `tools/test_priority_runtime.py` : cible à droite, traversée sans lignes, coupure caméra et reprise.
- `tools/test_priority_simulation.py` : launch ROS de la mission 2.
- `tools/test_navigation_modes_runtime.py` : boucle ROS sans SLAM, obstacle ajouté puis retiré et caméra explicite ; utilise uniquement des topics de simulation.
- `tools/test_enclosure_slam.py` : workflow du banc SLAM ; refuse un serveur qui ne se déclare pas simulé.
- `tools/integration_ros.py` : lancement des cinq bancs historiques.

Les conteneurs de test isolent les simulations des topics matériels. Les cartes et captures dans `docs/images` sont synthétiques et ne représentent pas le terrain réel.

## Avant un essai physique

Vérifier la connexion, les domaines ROS distincts, les topics et TF des capteurs, la cadence et les horodatages, le rayon réel du montage et les marges. Pour l’enceinte, confirmer la localisation et la zone autorisée, puis placer les repères avant l’entrée, à l’intérieur et au-delà de la seconde ouverture.

Un chemin impossible ou inconnu impose l’arrêt. Les contrôles logiciels ne prouvent pas l’absence de collision dans toute situation physique. Une réussite en simulation ne vaut pas validation matérielle générale.

## Refonte de la console — 4 novembre 2026

180 tests passent dans ROS 2 Humble, avec topics de test isolés. Les ajouts vérifient la persistance des adresses des deux profils, la conservation de la connexion en cas d’erreur disque et le réglage de vitesse sans démarrage de processus moteur. Dans le navigateur en démonstration : brouillon puis application de vitesse en cm/s, IP distinctes, changement de profil et rechargement avec le robot 9 sélectionné. Ces essais n’attestent pas la réussite physique des missions.

## Missions 2 et 4 — vision et dimensions du terrain

199 tests couvrent notamment la séparation proche/lointain, les images répétées ou interrompues, la priorité LiDAR qui annule la commande, le passage aux trois images proches et les positions dans/hors de l’enceinte. Le décodage de l’image est partagé entre les deux fenêtres pour limiter la charge.

Banc ROS mission 2 : attente d’une cible venant de droite, départ après libération, traversée sans lignes, interruption caméra, lignes lointaines seules, puis reprise proche. Banc mission 4 : enceinte physique synthétique de 1 × 1,25 m, départ dehors, SLAM réel, sauvegarde/rechargement, obstacle ajouté, passage intérieur et sortie dehors avant la limite de temps. La navigation sans SLAM avec obstacle reste également exercée.

Interface : repère métrique au clic. Les blocages géométriques du cadre, introduits dans cette version, ont été retirés ensuite à la demande de l’équipe : les dimensions sont indicatives. Les fenêtres proches/lointaines n’affectent que le pilote de mission 2. Essais matériels et calibration caméra/LiDAR encore nécessaires ; aucun robot réel n’a été modifié pendant ce développement.


## Barème officiel, dimensions indicatives et manuel — 4 novembre 2026

- **198 tests ROS passent**, dont un test recevant réellement les commandes manuelles sur un topic isolé : avance, recul, relâchement et arrêt après expiration de 250 ms.
- `node tools/test_console_ui.js` reproduit l’invalidation du manuel par une ancienne réponse HTTP sur la version précédente et passe après correction. Il vérifie aussi STOP pendant une activation, la reprise après rechargement, un cadre absent/modifié et le blocage sur capteurs périmés.
- Banc SLAM sans réseau matériel : cartographie, graphe sauvegardé/rechargé, entrée/intérieur/sortie avec obstacle ajouté et cadre volontairement placé hors terrain. Le parcours se termine avant la limite, sans dépendre du cadre.
- Navigateur en démonstration : missions officielles 0 à 6 affichées, mission 5 en pause, activation du manuel et message de reprise après rechargement.

Les dimensions n’imposent plus une géométrie au planificateur. Les points libres, la zone autorisée, l’empreinte, la pose et les mesures fraîches restent requis. La détection automatique du panneau de tunnel reste absente ; l’usage du capteur pour ce point du barème est à clarifier avec les organisateurs. Aucun robot réel n’a été modifié ou déplacé ; un essai manuel physique est encore nécessaire après transfert volontaire.
