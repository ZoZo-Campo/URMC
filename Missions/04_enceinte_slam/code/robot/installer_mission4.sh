#!/usr/bin/env bash
set -euo pipefail
# Exécuter sur Ubuntu du robot ; aucune commande moteur.
sudo apt-get update
sudo apt-get install -y ros-humble-slam-toolbox ros-humble-tf2-ros ros-humble-navigation2 ros-humble-nav2-bringup python3-yaml
printf '\nSLAM et Nav2 installés. Revenir dans la console et cliquer Connecter / préparer.\n'
