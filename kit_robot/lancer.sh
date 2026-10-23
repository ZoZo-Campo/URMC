#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "$0")/environnement.sh"
mission="${1:-}"; mode="${2:-observation}"
case "$mission" in 01|02|03|04) ;; *) echo 'Usage : bash kit_robot/lancer.sh 01|02|03|04 observation|roulage'; echo 'Mission 05 bloquée : intégration matérielle manquante.'; exit 1;; esac
case "$mode" in
 observation) output="/urmc/test/m$mission/cmd_vel" ;;
 roulage) output="$CMD_TOPIC" ;;
 *) echo 'Mode invalide'; exit 1;;
esac
[[ "$IMAGE_TRANSPORT" == raw || "$IMAGE_TRANSPORT" == compressed ]] || { echo 'Transport caméra invalide'; exit 1; }
python3 "$WS_DIR/kit_robot/suivi_piste_camera.py" --prepare-only
echo "Mission $mission — $mode — sortie $output — désarmée au démarrage"
echo 'Mission 03 : traversée tout droit uniquement. Arrêter les autres contrôleurs avant roulage.'
exec ros2 run "urmc_mission_$mission" mission --ros-args  -r "__ns:=/urmc/m$mission"  --params-file "$WS_DIR/src/urmc_mission_$mission/config/mission.yaml"  -p simulation:=false -p auto_arm:=false -p speed:=0.04  -p storage_calibrated:=false -p stop_shape_enabled:=false -p phase:=checker -p route:=straight  -p "image_transport:=$IMAGE_TRANSPORT"  -r "scan:=$SCAN_TOPIC" -r "odom:=$ODOM_TOPIC"  -r "image_raw:=$IMAGE_TOPIC" -r "image_raw/compressed:=$IMAGE_TOPIC"  -r "cmd_vel:=$output" -r "magnet_request:=/urmc/test/m$mission/magnet_request"
