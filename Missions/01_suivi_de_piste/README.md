# Mission 0 — Suivi de piste

Numéro officiel : **0**. Le dossier `01_suivi_de_piste` et le package `urmc_mission_01` gardent leurs noms techniques pour préserver les commandes existantes.

1. Ouvrir **OUVRIR.command**, choisir le robot puis **Connecter / préparer**.
2. **Missions → Suivre la piste → Robuste → Lancer le suivi**.
3. La vidéo montre les barycentres et le centre de piste. STOP termine l’essai.

Les deux bordures jaune et blanche sont requises, quel que soit leur côté. Le mode robuste s’arrête quand la perception est insuffisante et confirme plusieurs images avant la reprise. Le suivi historique reste limité à 90 secondes. Le pilote caméra n’évite pas les obstacles au LiDAR.

Le banc ROS propose une piste courbe et des perturbations : **Simulations → 0 → Ouvrir → Démarrer**. Les fonctions de recherche de ligne et de détour de ce banc ne sont pas toutes présentes dans le pilote caméra réel.

## État et points à gagner

- **4 points :** suivre la piste et s’arrêter au damier. Pilote préparé ; dernière version à revalider sur le terrain.
- **1 point :** virage puis arrêt au panneau STOP. **L’arrêt au panneau manque dans le pilote réel lancé par la console.** Le candidat de forme du banc générique ne le remplace pas.
- **Prochaine étape :** intégrer puis tester le STOP réel. Le bouton actuel lance le suivi et l’arrêt au damier.

Voir les [exigences et tests restants](../../docs/VALIDATION.md).

## Code

- `code/robot/suivi_piste_camera.py` : perception des deux bordures et essai historique.
- Le suivi robuste utilise le pilote partagé de la [mission 1](../02_barriere/code/robot/mission_barriere.py), avec `--ignore-barrier` ; il n’existe qu’une copie de ce pilote.
- `code/ros2/urmc_mission_01/` : logique et launch files du banc ROS.
- `tests/` : accès aux tests de perception et de reprise de ligne.

Les seuils sont dans l’interface → Réglages. Le code commun, la vidéo et le superviseur restent dans `../../kit_robot` et `../../src/urmc_core`.

## Règles résumées

- Perception par caméra embarquée. Ligne jaune à gauche, blanche à droite.
- Partie A : départ sur un cercle blanc, environ 1,5 m en ligne droite ; arrêt au passage du damier transversal.
- Partie B : ligne droite suivie d’une courbe ; arrêt à la détection du panneau STOP, annoncé à 5 × 5 cm.
- Le texte indique 20 cm de piste et des bandes de 2 cm. La figure 2 indique aussi 25 cm : dimensions de mesure à clarifier.
- La figure 2 illustre une interruption de ligne de 50 cm. Les conditions exactes d’application ne sont pas détaillées.

Barème : 4 points pour ligne droite + damier ; 1 point pour virage + STOP. −1 en cas de sortie ou de non-arrêt pour chaque partie.

Les [PDF officiels fournis](../../docs/documents/) restent la référence.

## Régler les couleurs dans la console

Dans **Paramètres**, les curseurs **Régler les couleurs de la piste** sont regroupés. Ils modifient un brouillon dans le navigateur ; **Appliquer au robot** est le seul bouton qui envoie ces réglages. **Annuler les changements** rétablit les valeurs enregistrées. Vitesse et direction sont dans le panneau dépliable.

La version actuellement installée adapte le seuil blanc à la lumière : le curseur de luminosité fixe son plancher. Les curseurs utilisent les paramètres déjà présents sur le robot ; leur ajout dans la console ne nécessite aucun transfert de programme. Le suivi seul utilise `--ignore-barrier` et ne prend aucune décision sur le bleu.
