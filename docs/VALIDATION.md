# Validation au 22 octobre 2026

## Tests reproductibles

- Six packages ROS 2 Humble compilés dans Ubuntu ARM64 sous Docker.
- 96 tests unitaires passent : logique, perception, protection des commandes, barrière bleue, deux bordures dans les deux ordres, rejet des sauts de barycentres et serveur HTTP vidéo.
- Le test HTTP vérifie l’annotation, le refus d’une image périmée et d’un message ROS vieux de 30 s.
- Le serveur permanent choisit les annotations selon la mission active et reste indépendant des contrôleurs.
- `tools/test_barrier_runtime.py` utilise de vrais nœuds ROS, un réseau Docker isolé et des vitesses remappées sur `/urmc_test/barrier/cmd_vel`. Il vérifie le suivi avant/après barrière, la fermeture, l’ouverture stable, les vues ambiguës, une nouvelle fermeture, les pertes caméra/piste, la pause et reprise après un écart excessif, le plafond de 5 cm/s et la durée facultative.
- Les cinq scénarios nominaux synthétiques avaient atteint DONE lors de la validation précédente. Les derniers changements concernent principalement les scripts expérimentaux ; ils ne constituent pas une nouvelle validation matérielle de toutes les missions.

Après compilation :

```bash
python3 -m pytest -q -p no:cacheprovider tests
python3 tools/test_barrier_runtime.py
```

Sur Mac : `bash tools/mac.sh test` exécute la suite de simulation. Le test du script barrière ci-dessus est une vérification supplémentaire à lancer dans ROS Humble, réseau isolé.

## Essais matériels

Le suivi avec virage a été confirmé par l’opérateur. Le 22 octobre, les journaux du robot montrent un suivi à environ 3 puis 5 cm/s commandés, un arrêt devant une barrière fermée et une reprise après ouverture confirmée. Ces essais étaient limités à 30 s, avant retrait de cette limite à la demande de l’opérateur.

Un essai prolongé a ensuite révélé des pauses sur une vue partielle du panneau et un arrêt définitif « piste trop décalée ». Les correctifs ajoutent le filtrage des barycentres et une pause récupérable avec trois images de confirmation. Un bleu ambigu sans fermeture observée ne verrouille plus définitivement la mission. La nouvelle version est installée ; sa robustesse sur un parcours complet reste à confirmer.

La vidéo a été vérifiée dans Chrome : images renouvelées, aucune erreur JavaScript, interruption signalée. Sur un échantillon, transferts médians d’environ 117 ms pour le suivi et 133 ms pour la barrière. Ces chiffres ne mesurent pas le délai caméra-écran synchronisé et varient avec le réseau.

## Limites

Le damier et le STOP réels, la priorité avec plusieurs robots, une enceinte arbitraire et la prise réelle de palette ne sont pas validés. Les prototypes caméra du menu ne gèrent pas les obstacles LiDAR. La simulation ne reproduit pas la caméra calibrée, le glissement, le réseau de salle, les collisions dynamiques ou la préhension physique. Aucun résultat de compétition n’est garanti.
