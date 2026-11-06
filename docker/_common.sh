#!/usr/bin/env bash
# Shared helpers for the run_*.sh scripts. Not meant to be executed directly.
set -euo pipefail

DOCKER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE=(docker compose -f "${DOCKER_DIR}/docker-compose.yml")

# Cache directories must exist and be writable by the container user (1234).
mkdir -p "${DOCKER_DIR}"/cache/{main,computecache,logs,config,data,pkg}
if [[ "$(stat -c %u "${DOCKER_DIR}/cache/main")" != "1234" ]]; then
  echo "[info] Giving the container user (uid 1234) ownership of docker/cache (may ask for sudo)."
  sudo chown -R 1234:1234 "${DOCKER_DIR}/cache"
fi

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <script.py> [script args...]" >&2
  exit 2
fi
