#!/bin/bash
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
export PATH="/opt/homebrew/bin:/usr/local/bin:$PATH"
command -v docker >/dev/null || { echo 'Docker est nécessaire. Voir LIRE_MOI.html.'; exit 1; }
if ! docker info >/dev/null 2>&1; then
  command -v colima >/dev/null || { echo 'Démarre Docker avant de continuer.'; exit 1; }
  colima start
fi
if ! docker image inspect urmc-humble:local >/dev/null 2>&1; then
  docker build -t urmc-humble:local "$ROOT_DIR"
fi
mode="${1:-menu}"
if [[ "$mode" == menu ]]; then
  echo 'Choisis : 1 suivi/damier, 2 barrière, 3 priorité, 4 enceinte LiDAR, 5 stockage'
  read -r answer
  case "$answer" in 1|2|3|4|5) mode="0$answer";; *) echo 'Choix invalide.'; exit 1;; esac
fi
name="urmc-local-$$"
cleanup(){ docker stop -t 5 "$name" >/dev/null 2>&1 || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM
if [[ "$mode" == test ]]; then
  docker run --rm --name "$name" --network none -v "$ROOT_DIR:/project:ro" urmc-humble:local bash /project/tools/container_run.sh test
else
  case "$mode" in 01|02|03|04|05) ;; *) echo 'Mission invalide';exit 1;; esac
  if curl --silent --max-time 1 http://127.0.0.1:8765/ >/dev/null 2>&1; then
    echo 'Le port 8765 est déjà utilisé. Arrête la simulation précédente avec Ctrl+C.'
    exit 1
  fi
  docker run --rm --name "$name" -p 127.0.0.1:8765:8765 -v "$ROOT_DIR:/project:ro" urmc-humble:local bash /project/tools/container_run.sh "$mode" &
  task_pid=$!
  for ((i=0;i<90;i++));do
    if ! kill -0 "$task_pid" 2>/dev/null;then wait "$task_pid";exit 1;fi
    if curl --silent --fail http://127.0.0.1:8765/ >/dev/null;then
      open -a 'Google Chrome' http://127.0.0.1:8765/
      break
    fi
    sleep 1
  done
  echo 'Simulation en cours. Ctrl+C pour arrêter les nœuds et le conteneur.'
  wait "$task_pid"
fi
