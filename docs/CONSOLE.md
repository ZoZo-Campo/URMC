# Console URMC

## Utilisation

Ouvrir `OUVRIR URMC.command`, sélectionner le robot puis **Connecter / préparer**. Le lancement et la connexion ne commandent aucun mouvement. La console reste sur `http://127.0.0.1:8090/`.

- **Piloter** : vidéo, LiDAR, maintien des flèches pour le manuel et raison de l’état courant.
- **Missions** : cartes 01 à 04, avec accès au panneau Enceinte / SLAM et raccourci d’entraînement sur le Mac ; mission 5 indiquée en simulation.
- **Enceinte / SLAM** : préparation guidée en trois étapes, progression visible et prochaine action indiquée : choix SLAM ou vue locale, parcours complet ou arrivée au clic, puis position/départ. Sans SLAM, la pose est relative à l’odométrie de départ et les déplacements sont limités à 3 m. Les commandes de départ restent désactivées si la carte, les repères, la position ou les capteurs ne sont pas prêts.
- **Simulations** : cinq bancs ROS synthétiques ; le banc SLAM dédié est dans Enceinte / SLAM.
- **Réglages** : perception, vitesse, seuils de priorité et gabarit/marge de l’enceinte.
- **Mode d’emploi** : accès aux documents de référence et étapes de connexion.

Les README des missions sont les guides détaillés. La mission 5 reste en simulation.

## Deux robots

Profils distincts : 11 à `.211`, domaine proposé 41 ; 9 à `.209` à confirmer, domaine proposé 39. Chaque profil garde ses réglages, sa connexion et ses tunnels. Le domaine des pilotes est vérifié lors de la préparation ; une adresse ou un domaine identique sur les deux profils est refusé.

Changer de robot demande l’arrêt au précédent et invalide le manuel. Les commandes portant l’ancienne sélection sont refusées. STOP TOUS s’adresse aux superviseurs connus et au banc SLAM actif. Une absence de réponse est signalée comme arrêt non confirmé.

## Code et supervision

`tools/operator_console.py` sert la page Mac ; `tools/console/` contient le navigateur. Les POST exigent le jeton de la page et une origine locale. Aucune commande shell libre n’est acceptée.

Le Mac utilise SSH pour transférer `src` et `kit_robot`, en matérialisant les liens vers les dossiers de mission, puis compile les packages requis. L’agent du robot (`kit_robot/operator_agent.py`) écoute seulement localhost:8091 et exige un jeton privé. Celui-ci n’est pas transmis au navigateur. Le robot reçoit un workspace autonome.

Un seul pilote possède les moteurs. Le manuel expire après 250 ms sans renouvellement ; une ancienne pression ne peut pas redémarrer le mouvement. Le superviseur attend la fin du programme précédent et les verrous communs empêchent les lancements concurrents. Les tests ROS remappent les vitesses sur des topics de test.

Le worker SLAM de la mission 4 écoute localhost:8093 derrière l’agent. Il ne crée aucun publisher de vitesse à l’observation ; le lancement de mission lui donne les moteurs. STOP publie zéro, détruit ce publisher et libère les verrous. Le SLAM reste disponible pour observer et préparer un autre essai.

## Retours capteurs

La vidéo sert toujours la dernière image, sans accumulation de flux. Les images périmées sont refusées. Barycentres, centre de piste, surfaces bleues et masques sont annotés. La carte LiDAR affiche les points, les secteurs et les cibles de priorité. Chaque réponse identifie son robot.

La mission 4 superpose la carte enregistrée, la pose TF, les obstacles LiDAR actuels, l’empreinte et le chemin recalculé. Sans SLAM, une vue locale utilise l’odométrie de départ. Une couche LiDAR bloque les nouveaux obstacles et confirme la disparition d’anciens obstacles après trois scans distincts libres ; la carte enregistrée reste intacte. Par défaut aucun abonnement caméra n’est créé. Vidéo et assistance caméra sont optionnelles, hors épreuve officielle : qualité d’image et changements à l’arrêt, sans reconnaissance d’obstacles ni estimation de distance.

## Fichiers locaux et publication

Le code canonique spécifique est dans `Missions/*/code`. Les liens dans `src` et `kit_robot` gardent les interfaces techniques existantes. `Missions/COLCON_IGNORE` empêche une double découverte ; colcon passe par `src`. Les conteneurs et les archives de transfert suivent ces liens.

Les cartes, graphes SLAM, réglages de session, jetons, journaux, résultats générés et environnements compilés sont exclus de Git. Les archives et références anciennes restent hors du dossier actif. Le dépôt publié et le dossier Code du Bureau sont maintenant une seule copie Git.
