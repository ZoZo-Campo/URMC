# Mission 2 — Barrière bleue

1. Ouvrir **OUVRIR.command**, choisir le robot puis **Connecter / préparer**.
2. **Missions → Franchir la barrière → Observer** pour vérifier le panneau.
3. **Lancer avec moteurs** pour suivre la piste, attendre la levée et continuer. STOP termine l’essai.

La barrière est un rectangle bleu, horizontal quand elle est fermée et vertical quand elle est levée. La plus grande composante bleue est B1 ; son orientation et sa forme doivent être cohérentes. Une composante plus grande non orientable ne peut pas être remplacée par un petit rectangle classé ouvert.

La vidéo affiche surfaces, angle, contours, barycentres et masques couleur. Les réglages de teinte, surface minimale, dominance et confirmation sont dans l’interface. Une ouverture validée ou un mouvement d’ouverture suivi d’une disparition confirmée autorise la reprise ; une simple absence initiale n’est pas une preuve d’ouverture.

Le suivi garde les deux bordures, dans les deux sens de couleur. Il n’a pas de durée totale maximale par défaut ; arrêt au damier ou STOP. L’odométrie et les images doivent rester récentes.

## Code

- `code/robot/mission_barriere.py` : pilote réel et suivi robuste partagé.
- `code/perception/barrier.py` : masque, surfaces, moments et confirmation de levée.
- `code/ros2/urmc_mission_02/` : logique et launch files du banc ROS.
- `tests/` : tests du détecteur.

Simulation : **Simulations → 2 → Ouvrir → Démarrer**. Les dimensions, l’éclairage et les seuils restent à vérifier sur le panneau de compétition.

## Règles résumées

- Suivre la piste, détecter la barrière fermant le passage et s’arrêter devant elle.
- Attendre son ouverture puis reprendre le suivi de piste.
- Le schéma montre une barrière rouge/blanche ; le texte ne fixe pas les dimensions ni la hauteur.

Barème : 3 points : détection et arrêt ; 2 points : reprise après ouverture. −1 si la barrière est heurtée.

Les [PDF officiels fournis](../../docs/documents/) restent la référence.
