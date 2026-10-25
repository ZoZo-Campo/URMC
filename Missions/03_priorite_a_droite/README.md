# Mission 3 — Priorité à droite

Sur le robot, utiliser l’interface du Mac : **Missions → Priorité à droite**.
Commencer par **Observer caméra + LiDAR** pour vérifier les mesures, puis **Lancer avec moteurs** pour l’essai. Le retour caméra et la carte LiDAR sont dans **Piloter**. STOP termine l’essai ; aucun redémarrage automatique.

## Fonctionnement

1. Suivre les deux bordures jaune et blanche, quel que soit leur côté.
2. Après au moins six images alignées, repérer une ouverture du carrefour : largeur augmentée ou les deux bordures perdues sur trois images distinctes. La perte d’une seule bordure provoque une pause et ne suffit pas à autoriser une traversée. Une image caméra absente/périmée provoque un arrêt ; elle ne confirme pas un carrefour.
3. S’arrêter et mémoriser le cap d’approche. Surveiller au LiDAR la voie qui arrive de droite.
4. Attendre tant qu’une cible du gabarit attendu est dans cette voie et à moins du seuil de distance au robot. Confirmer la libération sur plusieurs scans pendant une seconde.
5. Traverser tout droit au cap mémorisé, corrigé par l’odométrie, même sans lignes. Reprendre le suivi après trois images valides des deux bordures et au moins 30 cm parcourus.
6. Continuer sur la piste après ce carrefour. Une nouvelle perte des lignes impose un arrêt ; ce programme traite un seul carrefour par lancement.

Par défaut : 5 cm/s en suivi, 3,75 cm/s en traversée, rayon de priorité 80 cm, secteur −120° à −20°, gabarit apparent 8–40 cm, distance de traversée limitée à 1 m. Ce sont des valeurs de départ à calibrer sur le robot, pas des dimensions du règlement. Arrêt si le cap dévie de plus de 0,5 rad, si l’écart latéral dépasse 12 cm, si un obstacle est trop proche devant ou si un capteur est périmé.

La distance affichée est mesurée depuis le LiDAR embarqué jusqu’aux points de la cible : ce n’est pas une distance entre les deux centres de robots. Le mode distance prend aussi en compte une cible immobilisée. **Exiger une approche observée = 1** active un filtre de mouvement ; en mode 0, c’est la zone, le gabarit et la distance qui décident. Le suivi compense le mouvement de notre robot avec l’odométrie. Une cible déjà vue en approche reste retenue si elle s’arrête ou disparaît brièvement.

## Voir et régler

La vidéo conserve les barycentres et les trois masques et ajoute l’état de priorité, les distances, la progression et un petit radar. La carte LiDAR sous la vidéo montre les mesures brutes, la zone de priorité et les cibles T1, T2… avec leur distance et leur mouvement. Rouge = cible retenue ; orange = analyse en cours ; gris = autre mesure.

Dans **Réglages → Priorité à droite** : distance objet–robot, secteur angulaire, limite avant de la voie, gabarit, filtre de mouvement, délai de libération et longueur maximale sans lignes. Vérifier qu’un objet physiquement à droite s’affiche à droite sur la carte. Corriger l’orientation LiDAR si nécessaire, à l’arrêt.

Le LiDAR 2D ne reconnaît pas sémantiquement un robot ou une personne. Une personne hors de la zone est ignorée pour la priorité ; une personne dans la zone et du même gabarit peut être retenue. Les obstacles devant et les objets très proches restent des motifs d’arrêt. Le gabarit apparent dépend de la hauteur du LiDAR et de la face visible du robot adverse.

## Code associé

- `code/robot/mission_priorite.py` : nœud ROS 2 matériel, abonnement au flux **compressé**, `/scan`, `/odom`, publication `/cmd_vel` et `/urmc/priority/status`.
- `code/perception/priority.py` : décisions et suivi des cibles, partagés avec la simulation.
- `../../kit_robot/vision_overlay.py` et `retour_video.py` : vidéo et lecture LiDAR permanente.
- `../../tools/operator_console.py` : interface et installation sur le robot sélectionné.

Le workspace complet reste nécessaire. L’interface le transfère et compile `urmc_core` et les missions 01/02/03 ; ne copier qu’un fichier Python ne suffit pas.

En SSH sur le robot préparé, sans interface :

```bash
# Observation, aucune commande moteur :
python3 ~/urmc_essais_20261020/kit_robot/mission_priorite.py
# Un essai avec moteurs, Ctrl+C pour terminer :
python3 ~/urmc_essais_20261020/kit_robot/mission_priorite.py --run
```

Sur le robot 9, définir `URMC_ROBOT_DOMAIN=39 URMC_ROBOT_ID=9` avant la commande, après confirmation du domaine des pilotes. Le profil de l’interface transmet ces valeurs automatiquement. Les verrous partagés empêchent le lancement simultané d’une autre mission caméra/manuelle.

## Simulation sur le Mac

**Simulations → 3 · Priorité à droite → Ouvrir → Démarrer.** La caméra synthétique contient une ouverture puis une vraie zone sans lignes. « Maintenir le robot à droite » prolonge l’attente ; désactiver pour libérer. Ces parcours sont inventés pour tester la logique et ne reproduisent pas une scène physique officielle.

Après compilation dans ROS 2 Humble :

```bash
ros2 launch urmc_mission_03 sim.launch.py
```

Le lanceur ROS générique `robot.launch.py route:=straight` reste disponible, désarmé au départ, avec service `/urmc/m03/enable`. Pour les essais sur le robot, préférer la console et le nœud `mission_priorite.py`, qui ajoutent le suivi caméra robuste, les réglages à chaud et le retour opérateur. Le simulateur et le nœud matériel partagent le même contrôleur de priorité ; leurs adaptateurs caméra sont différents.

## Règlement et état

PDF français, page 4 : section 4.1.3 et figure 3. Priorité au robot venant de droite en cas d’arrivée simultanée. Le schéma n’indique pas de marque spécifique à l’entrée ni de longueur de traversée. Les 50 cm de la figure 2 décrivent une interruption de bordure différente. La détection d’entrée et les seuils ci-dessus sont nos choix de mise en œuvre.

Développé et testé dans ROS 2 simulé. Calibration du carrefour, du gabarit adverse, de l’orientation LiDAR et essais réels sur les robots 9 et 11 à effectuer. Aucune installation ni commande moteur réelle pendant ce développement.

## Règles résumées

- À l’arrivée simultanée de deux robots, appliquer la priorité à droite.
- Dans la figure 3, le robot 2 situé à droite du robot 1 est prioritaire.
- Le barème pénalise une mauvaise direction sans détailler complètement comment la direction est imposée.

Barème : 3 points : arrêt après détection du robot à droite ; 2 points : suivi avec priorité. −1 si pas de détection, −2 en cas de mauvaise direction.

Les [PDF officiels fournis](../../docs/documents/) restent la référence.
