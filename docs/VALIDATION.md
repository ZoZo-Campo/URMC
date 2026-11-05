# Exigences, points et état des missions

Numéros du barème officiel. **Les points sont attribués par le jury** : l’interface affiche les points possibles, sans les considérer comme déjà gagnés. Une réussite sur capteurs simulés ne valide pas l’épreuve sur le robot réel.

## Mission 0 · Suivi de piste · 5 points possibles

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Suivre la piste et s’arrêter au damier transversal | 4 | Pilote réel de suivi et arrêt au damier préparés ; essais de piste déjà réalisés, dernière chaîne à revalider | Ligne droite, damier, lumière et vitesse réelles |
| Suivre le virage et s’arrêter au panneau STOP | 1 | **Intégré** aux pilotes robuste et historique, choix damier / STOP / automatique, confirmé sur images distinctes | Tester le panneau réel et régler surface/contraste ; vérifier l’arrêt après le virage |

Malus : −1 en cas de sortie de piste ou d’arrêt manqué, pour chacune des deux parties.

Deux bordures jaune et blanche obligatoires, quel que soit leur côté. La version robuste confirme les images avant de reprendre après une pause. Le bleu n’intervient pas dans cette mission. Le pilote réel utilise maintenant un détecteur de forme rouge avec motif blanc central : octogone STOP et panneau rouge à barre blanche illustré dans les documents. Un rond rouge plein ou une boîte rouge ne suffisent pas. Les tests emploient du JPEG et plusieurs éclairages ; aucune reconnaissance de texte n’est revendiquée. Le marqueur d’arrêt et sa confirmation restent verrouillés une fois reconnus, sans redémarrage automatique. Un candidat trop petit ralentit l’approche au lieu de provoquer l’arrêt final ; la surface de déclenchement est réglable séparément de la détection. Le choix de cible est propre à l’essai et ne modifie pas les réglages partagés.

## Mission 1 · Barrière · 5 points possibles

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Détecter la barrière et s’arrêter avant contact | 3 | Détection du rectangle bleu et arrêt intégrés | Surface/angle, distance d’arrêt et vitesse sur la vraie barrière |
| Reprendre le suivi après l’ouverture | 2 | Confirmation de levée et reprise intégrées | Levée, immobilisation, disparition et reprise sur les deux bordures |

Malus : −1 si le robot heurte la barrière. Plus grande surface bleue, forme et orientation sont contrôlées. Une absence initiale de bleu ne suffit pas à déclarer la barrière ouverte. Le rectangle bleu, horizontal fermé et vertical levé, correspond à la description du matériel donnée par l’équipe.

## Mission 2 · Priorité à droite · 5 points possibles

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Détecter le robot arrivant à droite et s’arrêter | 3 | Zone, distance, gabarit et attente au LiDAR intégrés ; arrêt aussi avant la perte des lignes quand une cible est confirmée | Deuxième robot réel, orientation du LiDAR et réglage de la voie |
| Reprendre et traverser dans la bonne direction | 2 | Maintien du cap, zone sans lignes, vision lointaine puis reprise proche testés en simulation | Carrefour réel et reprise des deux bordures proches |

Malus : −1 si pas de détection ; −2 si mauvaise direction. La détection utilise la distance au robot et la voie de droite, pas l’ensemble de la pièce. Elle ne garantit pas la distinction entre un robot et une personne de même gabarit dans cette voie. Un seul carrefour est traité par lancement.

## Mission 4 · Enceinte · 7 points et bonus de 3

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Détecter le panneau d’entrée au tunnel | 1 | **Détecteur visuel préparé hors compétition**, candidat géométrique d’arche ; règle de capteur à clarifier | Faire préciser le capteur autorisé, puis tester le pictogramme réel ; ne pas compter ce point avant validation |
| Entrer dans l’enceinte sans toucher les montants | 3 | Approche A, confirmation de deux montants sur plusieurs scans, largeur mesurée puis franchissement ; avec et sans SLAM | Approche et largeur réelle d’entrée, rayon et marge |
| Trouver la sortie et ressortir dans le temps imparti | 3 | Navigation B → C, arrêt et chrono testés en simulation | Localisation et point C réellement à l’extérieur |
| Réussir avec les obstacles ajoutés | +3 | Ajout/retrait d’obstacles par scans et recalcul testés en simulation | Obstacles réels dans plusieurs positions |

Malus : −1 si une paroi est touchée ; −1 si un obstacle tombe. Le texte impose le LiDAR seul et interdit la caméra durant cette mission. Le panneau dessiné en figure 6 représente une arche ; le point du barème ne peut pas être déclaré validé par un repère A placé à la main.

Le robot commence réellement dehors ; A est avant l’entrée, B dedans, C dehors après la sortie. **Le pilote utilise les repères préparés pour chercher au bon endroit**, puis exige une ouverture réellement mesurée au LiDAR. Un seul montant, une largeur insuffisante, des scans répétés ou un obstacle dans le passage empêchent sa confirmation. La cible de franchissement est calculée après le plan des montants, avec le rayon et la marge courants. Le panneau visuel reste distinct de cette détection de passage. La zone autorisée doit empêcher un détour extérieur.

SLAM au choix. Le LiDAR continue d’analyser les obstacles pendant le parcours. **1 × 1,25 m est indicatif**, ajustable et facultatif, sans contrainte sur le trajet. Les murs viennent des mesures. Rayon, marge et vitesse sont réglables. Sans SLAM : origine odométrique et objectifs à moins de 3 m ; dérive possible. Le chrono logiciel de 60 s démarre après la confirmation de l’ouverture, avant son franchissement : cela reste conservateur par rapport à la minute à l’intérieur. En mode trajet direct, il commence au départ.

## Missions laissées de côté

- **3 · Collectif piste :** 8 points avant classement, −1 par position d’arrivée selon le barème. Programme collectif à préparer.
- **5 · Palette :** en pause à la demande de l’équipe. Aucun lancement dans l’interface ni nouveau développement. Ancienne simulation conservée ; reconnaissance des zones et pilote de l’électroaimant absents.
- **6 · Collectif manœuvres :** 8 points avant classement, −1 par position d’arrivée selon le barème. Programme collectif à préparer.

## Vérifications transversales restantes

- [ ] Tester physiquement la dernière commande manuelle : avance, recul, rotation, relâchement et STOP.
- [ ] Confirmer les IP actuelles et le domaine des pilotes des robots 9 et 11 ; profils séparés dans l’interface.
- [ ] Relever la cadence réelle caméra/LiDAR/odométrie, l’âge des images et la latence vidéo.
- [ ] Calibrer les seuils de couleur, la vitesse, le rayon, les marges et les distances sur le terrain réel.
- [ ] Valider chaque critère du barème devant un observateur ; ne pas confondre programme disponible et points acquis.

## Preuves de validation disponibles

**226 tests ROS passent** dans la validation moteur/perception de cette version, sur topics isolés. Ils couvrent perception, fraîcheur, HTTP, réglages, profils distincts, verrous, grilles, chemins et commandes périmées. Le manuel a été testé avec un vrai abonné ROS sur un topic de test : avance, recul, relâchement et arrêt après 250 ms sans maintien. Aucune commande moteur réelle durant ces vérifications.

Le défaut d’activation manuelle a été reproduit sur l’ancienne interface : une réponse HTTP ancienne effaçait la nouvelle session. Le test `node tools/test_console_ui.js` passe après correction. Il vérifie aussi la reprise après rechargement, STOP pendant une activation, le cadre indicatif facultatif et les capteurs périmés.

Le banc SLAM réalise cartographie, sauvegarde/rechargement du graphe, pose confirmée, entrée/intérieur/sortie et contournement d’un obstacle ajouté après cartographie. Le parcours reste valide avec un cadre indicatif volontairement placé hors du terrain. Le banc sans SLAM vérifie l’arrivée choisie et l’actualisation d’obstacles. Ce sont des capteurs synthétiques, pas les robots réels.

La page Missions distingue maintenant les objectifs du barème, les comportements préparés, les manques et le programme réellement actif. Les pilotes de cette version ont évolué : arrêt au STOP, confirmation fraîche des marqueurs, attente anticipée à droite et entrée mesurée. Le transfert et les essais sur les robots réels restent à faire ; aucun robot n’a été modifié pendant ce développement.

## Résultats de cette version

- Pilotes réels sur ROS isolé : suivi robuste avec arrêt STOP, suivi robuste avec arrêt damier, suivi historique avec arrêt STOP. Le mauvais marqueur et les formes rouges sans motif blanc ne terminent pas l’essai.
- Barrière : arrêt, levée stable, reprise, fermeture répétée, perte de piste et de caméra, sans durée globale imposée ; images et odométrie doivent rester fraîches à la fois à la réception et à la capture.
- Priorité : attente et libération, traversée sans lignes, interruption caméra, fenêtres lointaines et reprise proche. Le nouvel arrêt d’approche ne marque pas le carrefour comme déjà franchi.
- Enceinte : SLAM réel sur scans simulés, graphe sauvegardé/rechargé, obstacle ajouté et entrée mesurée ; parcours complet sans SLAM également exercé. Aucun capteur simulé ne commande un topic moteur matériel.
- Caméra du tunnel : reconnaissance du pictogramme de l’illustration et tests synthétiques, confirmation spatiale et temporelle. Désactivée par défaut ; pas d’abonnement caméra créé par le lancement LiDAR. Une reconnaissance illustrée ne prouve pas celle du panneau réel.
- Configuration illisible : les vitesses caméra **et** enceinte passent à zéro. Le banc Mac initialise explicitement son fichier de réglages. Les files caméra utilisent le dernier message, profondeur 1.

## Outils de vérification

- `python3 -m pytest` : tests ROS, sous Ubuntu/ROS 2 Humble.
- `node tools/test_console_ui.js` : ordre des réponses et commande manuelle de l’interface.
- `python3 tools/check_layout.py` : dossiers, packages et liens internes.
- `tools/test_finish_runtime.py` : choix du marqueur, rejet des faux panneaux et arrêt final des deux pilotes.
- `tools/test_barrier_runtime.py` : barrière sur capteurs ROS synthétiques.
- `tools/test_priority_runtime.py` : priorité, traversée sans lignes et reprise proche.
- `tools/test_navigation_modes_runtime.py --enclosure` : parcours complet sans SLAM et entrée mesurée ; sans argument, trajet direct.
- `tools/test_enclosure_slam.py` : graphe SLAM, entrée/sortie et cadre indicatif ; refuse un serveur non simulé.

Les [documents officiels fournis](documents/) restent la référence. Les chemins techniques `01`, `02`, `03` correspondent aux missions officielles **0**, **1**, **2** et restent stables pour les robots.

## Mission 4 — Nav2, 5 novembre 2026

230 tests unitaires passent dans ROS 2 Humble, ainsi que les vérifications JavaScript et des chemins. Parcours réel ROS simulé validé : carte créée, graphe sauvegardé/rechargé, pose confirmée, entrée mesurée au LiDAR, obstacle ajouté après cartographie contourné, intérieur et sortie atteints puis arrêt. Le parcours complet sans SLAM reste validé, avec suppression d’obstacle et caméra sur activation.

Nav2 utilise NavFn A* et Regulated Pure Pursuit, avec carte globale SLAM et carte locale LiDAR/odom distinctes. Les vitesses passent exclusivement par le superviseur de mission. Annulation et commandes tardives testées ; trois échecs de suivi maximum avant arrêt. Le simulateur publie maintenant aussi les vitesses odométriques.

Les essais de roulage avec Nav2 sur le robot physique restent à faire avec l’opérateur ; une simulation réussie ne valide pas la compétition réelle.

Déploiement sur robot 11 : dépendances installées et cinq paquets compilés. Le pilote OpenCR avait perdu la communication et s’était terminé ; après relance du bringup, la communication et l’odométrie sont revenues. Le lancement Nav2 sans préfixe a été corrigé et couvert par un test supplémentaire. Les deux serveurs Nav2 sont désormais actifs sur le robot physique, sans objectif envoyé, navigation inactive et pose non confirmée.

Contrôles à l’arrêt sur 10 secondes : caméra compressée décodable 320 × 240 (238 images), odométrie (169 messages), LiDAR 360° (42 scans), batterie 12,17 V. Transformations odom → base_footprint et base_footprint → base_scan valides. Aucune commande reçue sur /cmd_vel et aucun éditeur de commande moteur.

**Réserve avant roulage :** contrôle LiDAR sur 20 scans : 87,8 à 89,4 % de distances valides, environ 40 valeurs nulles sur 360, notamment entre 30° et 80°. Le LiDAR est déclaré dégagé par l’opérateur. Ces valeurs ne permettent pas de conclure à une panne ; leur cause reste à caractériser. Le superviseur conserve son seuil de qualité de 90 % et bloque le départ. Ne pas confondre « serveurs Nav2 prêts » et « mission prête à rouler ».

Les essais moteurs et l’évaluation sur piste restent à réaliser avec l’opérateur.

## Correction du pilotage pendant la cartographie — 5 novembre 2026

Deux causes identifiées sur robot 11 : les flèches SLAM étaient désactivées par le critère LiDAR de navigation autonome ; l’OpenCR remontait `torque=false`. Les flèches utilisent désormais le bail de commande manuel explicite (identique à la vue principale), et l’activation de chaque mode moteur vérifie le couple via `/motor_power` et `/sensor_state`. Les observations sans moteurs ne l’activent pas. Un appui maintenu reste nécessaire pour produire une vitesse non nulle.

La cartographie et la pose restent actualisées avec les retours LiDAR valides même si leur proportion ne permet pas le départ autonome. Le seuil de 90 % pour l’autonomie est conservé. Le diagnostic donne désormais le pourcentage et la couverture mesurés.

Validation : 236 tests unitaires passent, tests JavaScript des baux/commandes/états SLAM, test ROS isolé du superviseur jusqu’aux vitesses sur un topic de test avec relâchement, expiration et STOP. La sauvegarde/reprise du graphe en mode cartographie est validée sans déplacement dans le simulateur. Le banc exporte le même domaine ROS pour le simulateur et le moniteur.

Sur robot : couple actif confirmé sans éditeur de vitesse ; carte sauvegardée et reprise sans repartir de zéro ; position à nouveau récente (environ 0,1 s). Aucun essai de déplacement physique n’a été déclenché par l’assistant pendant cette correction.

Contrôle final dans le navigateur réel : bouton Conduire activé, couple confirmé actif, pilote en état MANUAL, quatre flèches disponibles malgré la qualité LiDAR insuffisante pour l’autonomie. Aucun appui directionnel déclenché ; commandes observées au repos. Le chargement ROS est réutilisé par les enfants du superviseur pour éviter les expirations de délai, et les flèches attendent le premier état MANUAL du pilote avant d’accepter un appui.
