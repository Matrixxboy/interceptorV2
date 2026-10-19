.PHONY: install install-air install-gcs install-dev lint typecheck test test-cov sitl-test run-air run-gcs clean

PYTHON ?= python3

# ── Install ─────────────────────────────────
install:
	$(PYTHON) -m pip install -e ".[dev]"

install-air:
	$(PYTHON) -m pip install -e ".[air,dev]"

install-gcs:
	$(PYTHON) -m pip install -e ".[gcs,dev]"

install-dev:
	$(PYTHON) -m pip install -e ".[air,gcs,dev]"
	pre-commit install

# ── Quality ─────────────────────────────────
lint:
	ruff check shared/ air-unit/ ground-station/
	ruff format --check shared/ air-unit/ ground-station/

lint-fix:
	ruff check --fix shared/ air-unit/ ground-station/
	ruff format shared/ air-unit/ ground-station/

typecheck:
	mypy shared/ air-unit/ --ignore-missing-imports

# ── Test ────────────────────────────────────
test:
	pytest

test-cov:
	pytest --cov=shared --cov=air-unit/core --cov=air-unit/control --cov=air-unit/vision \
	       --cov-report=term-missing --cov-report=html

sitl-test:
	pytest -m sitl --timeout=120

# ── Run ─────────────────────────────────────
run-air:
	$(PYTHON) air-unit/main.py

run-gcs:
	$(PYTHON) ground-station/main.py

# ── Clean ───────────────────────────────────
clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name .mypy_cache -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name .pytest_cache -exec rm -rf {} + 2>/dev/null || true
	rm -rf htmlcov/ .coverage
