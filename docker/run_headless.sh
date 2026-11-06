#!/usr/bin/env bash
# Run a Python script with Isaac Sim's interpreter, headless (no display).
# Example: ./docker/run_headless.sh scripts/smoke_test_hover.py
source "$(dirname "$0")/_common.sh"
"${COMPOSE[@]}" run --rm sim /isaac-sim/python.sh "$@"
