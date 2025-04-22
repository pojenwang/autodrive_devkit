#!/usr/bin/env bash
set -euo pipefail

# allow X11 forwarding
sudo xhost +

# move into script’s dir
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# parse --build/-b flag
BUILD=""
if [[ $# -gt 0 ]]; then
  case "$1" in
    -b|--build) BUILD="--build" ;;
    *) echo "Usage: $0 [--build|-b]"; exit 1 ;;
  esac
fi

echo "Pulling images…"
docker compose -f docker-compose-roboracer-sim.yml pull

echo "Bringing up services…"
docker compose -f docker-compose-roboracer-sim.yml up -d $BUILD --remove-orphans

echo "Status:"
docker compose -f docker-compose-roboracer-sim.yml ps
