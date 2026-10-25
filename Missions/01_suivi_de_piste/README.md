# Mission 1 — Suivi de piste

Cette épreuve était appelée « mission 0 » dans les premiers essais.

1. Ouvrir **OUVRIR.command**, choisir le robot puis **Connecter / préparer**.
2. **Missions → Suivre la piste → Robuste → Lancer le suivi**.
3. La vidéo montre les barycentres et le centre de piste. STOP termine l’essai.

Les deux bordures jaune et blanche sont requises, quel que soit leur côté. Le mode robuste s’arrête quand la perception est insuffisante et confirme plusieurs images avant la reprise. Le suivi historique reste limité à 90 secondes. Le pilote caméra n’évite pas les obstacles au LiDAR.

Le banc ROS propose une piste courbe et des perturbations : **Simulations → 1 → Ouvrir → Démarrer**. Les fonctions de recherche de ligne et de détour de ce banc ne sont pas toutes présentes dans le pilote caméra réel.

## Code

- `code/robot/suivi_piste_camera.py` : perception des deux bordures et essai historique.
- Le suivi robuste utilise le pilote partagé de la [mission 2](../02_barriere/code/robot/mission_barriere.py), avec `--ignore-barrier` ; il n’existe qu’une copie de ce pilote.
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
