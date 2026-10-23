#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "$0")/environnement.sh"
mission="${1:-}"; action="${2:-}"
case "$mission" in 01|02|03|04) ;; *) echo 'Mission autorisée : 01 à 04'; exit 1;; esac
case "$action" in
 armer) exec ros2 service call "/urmc/m$mission/enable" std_srvs/srv/SetBool '{data: true}' ;;
 arreter) exec ros2 service call "/urmc/m$mission/enable" std_srvs/srv/SetBool '{data: false}' ;;
 reset) exec ros2 service call "/urmc/m$mission/reset" std_srvs/srv/Trigger '{}' ;;
 etat) exec ros2 topic echo "/urmc/m$mission/status" ;;
 *) echo 'Usage : bash kit_robot/commande.sh 01 armer|arreter|reset|etat'; exit 1;;
esac
