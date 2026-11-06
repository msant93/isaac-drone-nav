#!/usr/bin/env bash
# Run a script with the Isaac Sim GUI on the local X11 display (local PC only).
# Example: ./docker/run_gui.sh scripts/smoke_test_hover.py
source "$(dirname "$0")/_common.sh"
script="$1"; shift
xhost +local: >/dev/null
trap 'xhost -local: >/dev/null' EXIT
"${COMPOSE[@]}" run --rm \
  -e DISPLAY="${DISPLAY}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  sim /isaac-sim/python.sh "${script}" --gui "$@"
