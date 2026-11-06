"""Sanity tests for the package skeleton (no Isaac Sim required)."""

from __future__ import annotations

import importlib
import re

import pytest

import dronenav


def test_version_is_semver() -> None:
    assert re.fullmatch(r"\d+\.\d+\.\d+", dronenav.__version__)


@pytest.mark.parametrize("name", dronenav.SUBPACKAGES)
def test_simulator_agnostic_subpackages_import(name: str) -> None:
    module = importlib.import_module(f"dronenav.{name}")
    assert module.__doc__, f"dronenav.{name} must have a module docstring"


def test_isaac_backend_not_in_agnostic_list() -> None:
    # The Isaac/Pegasus adapter must never be imported by simulator-agnostic code.
    assert "sim.isaac_pegasus" not in dronenav.SUBPACKAGES
