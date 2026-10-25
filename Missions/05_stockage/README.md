# Mission 5 — Stockage et transport de palette

**Simulation uniquement : le pilote réel de l’électroaimant et la reconnaissance des zones restent à intégrer.**

Ouvrir **OUVRIR.command**, puis **Simulations → 5 → Ouvrir → Démarrer**. Le banc fournit des observations synthétiques de zones, de palette et de prise. Ces observations ne constituent pas un détecteur matériel.

## Code

- `code/ros2/urmc_mission_05/urmc_mission_05/logic.py` : recherche, prise, transfert et libération.
- `code/ros2/urmc_mission_05/config/mission.yaml` : paramètres.
- Les topics et conditions d’armement sont décrits dans [les interfaces ROS](../../docs/INTERFACES.md).

Le mode réel reste bloqué tant que la localisation des zones, les confirmations de prise et le pilote de l’aimant ne sont pas intégrés et calibrés.

## Règles résumées

- Trois zones numérotées 1, 2, 3 par des panneaux, dans un ordre aléatoire.
- Une palette de 15 × 15 × 5 cm est initialement dans la zone 1, sur un support de hauteur 10 cm.
- Saisir sa partie métallique avant avec l’électroaimant Grove fourni.
- Transférer de la zone 1 à la zone 2 sans traverser d’autres zones et en conservant l’orientation initiale.
- Relâcher la palette au centre de la zone 2 puis sortir. La commande de l’électroaimant est laissée libre.

Barème : 5 points de base : prise 1, déplacement 3, libération 1. Bonus : détection/positionnement +2, sortie de zone +1. −1 si pas d’arrêt ; −1 si traversée d’une autre zone.

Les [PDF officiels fournis](../../docs/documents/) restent la référence.
