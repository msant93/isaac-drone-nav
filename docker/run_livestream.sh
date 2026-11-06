#!/usr/bin/env bash
# Run a script headless and stream the viewport over WebRTC.
# Connect with the "Isaac Sim WebRTC Streaming Client" to the host IP
# (for AWS: open the ports listed in docs/setup.md, or tunnel them over SSH).
# Example: ./docker/run_livestream.sh scripts/smoke_test_hover.py
source "$(dirname "$0")/_common.sh"
script="$1"; shift
"${COMPOSE[@]}" run --rm sim /isaac-sim/python.sh "${script}" --livestream "$@"
