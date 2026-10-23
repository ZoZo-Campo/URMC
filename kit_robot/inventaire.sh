#!/usr/bin/env bash
# Lecture seule ROS : aucun moteur, service d’armement ou installation.
set -eo pipefail
OUT="${1:-$HOME/urmc-inventaire-$(date +%Y%m%d-%H%M%S).txt}"
[[ ! -e "$OUT" ]] || { echo 'Rapport déjà existant, choisir un autre nom.'; exit 1; }
{
 date -Is
 uname -m
 cat /etc/os-release
 echo '--- Distributions présentes ---'
 ls /opt/ros 2>/dev/null || true
 echo '--- Environnement du terminal (à exécuter après configuration du bringup) ---'
 for name in ROS_DISTRO ROS_DOMAIN_ID ROS_LOCALHOST_ONLY RMW_IMPLEMENTATION TURTLEBOT3_MODEL; do printf '%s=%s
' "$name" "${!name:-non défini}"; done
 for tool in ros2 colcon rosdep; do command -v "$tool" || true; done
 if command -v ros2 >/dev/null; then
  timeout 12 ros2 node list || true
  timeout 12 ros2 topic list -t || true
  for topic in /scan /odom /image_raw /image_raw/compressed /cmd_vel; do
   echo "--- $topic ---"
   timeout 8 ros2 topic info "$topic" --verbose || true
  done
 fi
} 2>&1 | tee "$OUT"
echo "Rapport enregistré : $OUT"
