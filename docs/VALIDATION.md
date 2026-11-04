# Exigences, points et état des missions

Numéros du barème officiel. **Les points sont attribués par le jury** : l’interface affiche les points possibles, sans les considérer comme déjà gagnés. Une réussite sur capteurs simulés ne valide pas l’épreuve sur le robot réel.

## Mission 0 · Suivi de piste · 5 points possibles

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Suivre la piste et s’arrêter au damier transversal | 4 | Pilote réel de suivi et arrêt au damier préparés ; essais de piste déjà réalisés, dernière chaîne à revalider | Ligne droite, damier, lumière et vitesse réelles |
| Suivre le virage et s’arrêter au panneau STOP | 1 | **Incomplet** : le pilote caméra de la console ne prend pas la détection STOP en compte | Intégrer le détecteur au pilote, puis tester sur le panneau réel |

Malus : −1 en cas de sortie de piste ou d’arrêt manqué, pour chacune des deux parties.

Deux bordures jaune et blanche obligatoires, quel que soit leur côté. La version robuste confirme les images avant de reprendre après une pause. Le bleu n’intervient pas dans cette mission. Le banc générique contient un candidat de forme rouge ; cela ne constitue pas un arrêt STOP intégré et validé dans le pilote réel. Les anciens textes qui présentaient ce point comme acquis étaient trop affirmatifs.

## Mission 1 · Barrière · 5 points possibles

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Détecter la barrière et s’arrêter avant contact | 3 | Détection du rectangle bleu et arrêt intégrés | Surface/angle, distance d’arrêt et vitesse sur la vraie barrière |
| Reprendre le suivi après l’ouverture | 2 | Confirmation de levée et reprise intégrées | Levée, immobilisation, disparition et reprise sur les deux bordures |

Malus : −1 si le robot heurte la barrière. Plus grande surface bleue, forme et orientation sont contrôlées. Une absence initiale de bleu ne suffit pas à déclarer la barrière ouverte. Le rectangle bleu, horizontal fermé et vertical levé, correspond à la description du matériel donnée par l’équipe.

## Mission 2 · Priorité à droite · 5 points possibles

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Détecter le robot arrivant à droite et s’arrêter | 3 | Zone, distance, gabarit et attente au LiDAR intégrés ; tests ROS simulés passés | Deuxième robot réel, orientation du LiDAR et réglage de la voie |
| Reprendre et traverser dans la bonne direction | 2 | Maintien du cap, zone sans lignes, vision lointaine puis reprise proche testés en simulation | Carrefour réel et reprise des deux bordures proches |

Malus : −1 si pas de détection ; −2 si mauvaise direction. La détection utilise la distance au robot et la voie de droite, pas l’ensemble de la pièce. Elle ne garantit pas la distinction entre un robot et une personne de même gabarit dans cette voie. Un seul carrefour est traité par lancement.

## Mission 4 · Enceinte · 7 points et bonus de 3

| Pour gagner les points | Points | État actuel | Prochaine vérification |
|---|---:|---|---|
| Détecter le panneau d’entrée au tunnel | 1 | **Non développé** ; règle de capteur ambiguë | Faire préciser aux organisateurs la détection du panneau et l’interdiction caméra, puis développer |
| Entrer dans l’enceinte sans toucher les montants | 3 | Navigation A → B sur carte/vues LiDAR préparées, testée en simulation | Approche et largeur réelle d’entrée, rayon et marge |
| Trouver la sortie et ressortir dans le temps imparti | 3 | Navigation B → C, arrêt et chrono testés en simulation | Localisation et point C réellement à l’extérieur |
| Réussir avec les obstacles ajoutés | +3 | Ajout/retrait d’obstacles par scans et recalcul testés en simulation | Obstacles réels dans plusieurs positions |

Malus : −1 si une paroi est touchée ; −1 si un obstacle tombe. Le texte impose le LiDAR seul et interdit la caméra durant cette mission. Le panneau dessiné en figure 6 représente une arche ; le point du barème ne peut pas être déclaré validé par un repère A placé à la main.

Le robot commence réellement dehors ; A est avant l’entrée, B dedans, C dehors après la sortie. **Le pilote actuel utilise ces repères préparés** : il ne certifie pas automatiquement la détection du panneau ou des ouvertures. La zone autorisée doit empêcher un détour extérieur.

SLAM au choix. Le LiDAR continue d’analyser les obstacles pendant le parcours. **1 × 1,25 m est indicatif**, ajustable et facultatif, sans contrainte sur le trajet. Les murs viennent des mesures. Rayon, marge et vitesse sont réglables. Sans SLAM : origine odométrique et objectifs à moins de 3 m ; dérive possible. Le chrono logiciel de 60 s démarre au repère d’approche A, de façon conservatrice avant l’entrée complète.

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

**198 tests ROS passent** dans la dernière validation moteur/perception, sur topics isolés. Ils couvrent perception, fraîcheur, HTTP, réglages, profils distincts, verrous, grilles, chemins et commandes périmées. Le manuel a été testé avec un vrai abonné ROS sur un topic de test : avance, recul, relâchement et arrêt après 250 ms sans maintien. Aucune commande moteur réelle durant ces vérifications.

Le défaut d’activation manuelle a été reproduit sur l’ancienne interface : une réponse HTTP ancienne effaçait la nouvelle session. Le test `node tools/test_console_ui.js` passe après correction. Il vérifie aussi la reprise après rechargement, STOP pendant une activation, le cadre indicatif facultatif et les capteurs périmés.

Le banc SLAM réalise cartographie, sauvegarde/rechargement du graphe, pose confirmée, entrée/intérieur/sortie et contournement d’un obstacle ajouté après cartographie. Le parcours reste valide avec un cadre indicatif volontairement placé hors du terrain. Le banc sans SLAM vérifie l’arrivée choisie et l’actualisation d’obstacles. Ce sont des capteurs synthétiques, pas les robots réels.

La page Missions distingue maintenant les objectifs du barème, les comportements préparés, les manques et le programme réellement actif. La refonte de cette page ne modifie aucun algorithme moteur.

## Outils de vérification

- `python3 -m pytest` : tests ROS, sous Ubuntu/ROS 2 Humble.
- `node tools/test_console_ui.js` : ordre des réponses et commande manuelle de l’interface.
- `python3 tools/check_layout.py` : dossiers, packages et liens internes.
- `tools/test_barrier_runtime.py` : barrière sur capteurs ROS synthétiques.
- `tools/test_priority_runtime.py` : priorité, traversée sans lignes et reprise proche.
- `tools/test_navigation_modes_runtime.py` : navigation sans SLAM et obstacles actualisés.
- `tools/test_enclosure_slam.py` : graphe SLAM, entrée/sortie et cadre indicatif ; refuse un serveur non simulé.

Les [documents officiels fournis](documents/) restent la référence. Les chemins techniques `01`, `02`, `03` correspondent aux missions officielles **0**, **1**, **2** et restent stables pour les robots.
