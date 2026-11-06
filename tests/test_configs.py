"""Every YAML file in config/ and scenarios/ must parse; smoke config must be consistent."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
YAML_FILES = sorted([*ROOT.glob("config/**/*.yaml"), *ROOT.glob("scenarios/**/*.yaml")])


def _load(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    assert isinstance(data, dict), f"{path} must contain a mapping"
    return data


@pytest.mark.parametrize("path", YAML_FILES, ids=lambda p: str(p.relative_to(ROOT)))
def test_yaml_parses(path: Path) -> None:
    _load(path)


def test_smoke_config_is_consistent() -> None:
    cfg = _load(ROOT / "config" / "smoke_test.yaml")
    sim, mission, ctl, acc = cfg["simulation"], cfg["mission"], cfg["controller"], cfg["acceptance"]

    assert 0.0 < sim["physics_dt"] <= sim["rendering_dt"]
    assert mission["takeoff_altitude"] > cfg["vehicle"]["spawn_position"][2]
    assert mission["climb_rate"] > 0 and mission["descent_rate"] > 0
    for key in ("kp", "kd", "ki", "kr", "kw"):
        assert len(ctl[key]) == 3 and all(g >= 0 for g in ctl[key])

    # The whole profile (climb + hover + descent) must fit in the time budget.
    climb = (mission["takeoff_altitude"] - cfg["vehicle"]["spawn_position"][2]) / mission["climb_rate"]
    descent = mission["takeoff_altitude"] / mission["descent_rate"]
    assert climb + mission["hover_duration"] + descent < sim["max_sim_time"]
    assert acc["hover_settle_time"] < mission["hover_duration"]
