# Convenience targets. Host targets need the 'dronenav' conda env;
# docker targets need Docker + NVIDIA Container Toolkit.
.PHONY: help install check-env lint format typecheck test ci docker-build docker-shell smoke smoke-stream

help:
	@grep -E '^[a-z-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN{FS=":.*?## "}{printf "  %-14s %s\n",$$1,$$2}'

install: ## Install host dependencies and dronenav (editable)
	pip install -r requirements.txt && pip install -e .

check-env: ## Diagnose GPU/driver/Docker/disk on this machine
	python scripts/check_env.py

lint: ## Ruff lint + format check
	ruff check . && ruff format --check .

format: ## Auto-format the code
	ruff check --fix . && ruff format .

typecheck: ## mypy (strict) on src/
	mypy

test: ## Unit tests (no Isaac Sim needed)
	pytest

ci: lint typecheck test ## Everything CI runs

docker-build: ## Build the Isaac Sim + Pegasus image
	docker compose -f docker/docker-compose.yml build

docker-shell: ## Interactive shell inside the container
	docker compose -f docker/docker-compose.yml run --rm sim

smoke: ## Headless takeoff/hover/land smoke test in Isaac Sim
	./docker/run_headless.sh scripts/smoke_test_hover.py

smoke-stream: ## Same, with WebRTC livestream
	./docker/run_livestream.sh scripts/smoke_test_hover.py
