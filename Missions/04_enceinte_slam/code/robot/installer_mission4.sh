#!/usr/bin/env bash
set -euo pipefail
# Exécuter sur Ubuntu du robot ; aucune commande moteur.
sudo apt-get update
sudo apt-get install -y ros-humble-slam-toolbox ros-humble-tf2-ros python3-yaml
printf '\nSLAM installé. Revenir dans la console et cliquer Connecter / préparer.\n'
