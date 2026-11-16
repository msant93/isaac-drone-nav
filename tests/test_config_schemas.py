"""Tests for the typed configuration schemas and the YAML loader."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import pytest
import yaml

from dronenav.config.loader import ConfigError, load_config, load_yaml, parse_config
from dronenav.config.schemas import PythonSimConfig, VehicleConfig


def iris_dict() -> dict[str, Any]:
    return load_yaml("drone_iris.yaml")["vehicle"]


def test_iris_config_loads(iris: VehicleConfig) -> None:
    assert iris.num_rotors == 4
    assert iris.mass == pytest.approx(1.52)
    assert 2.0 < iris.max_thrust / (iris.mass * 9.81) < 3.5


def test_sim_config_loads(sim_config: PythonSimConfig) -> None:
    assert sim_config.physics_dt == pytest.approx(0.004)
    assert sim_config.integrator == "rk4"


@pytest.mark.parametrize(
    ("key", "value", "message"),
    [
        ("mass", -1.0, "greater than 0"),
        ("inertia", [0.01, 0.01, 0.5], "triangle inequality"),
        ("inertia", [0.01, 0.01], "at least 3"),
        ("max_rotor_velocity", 300.0, "thrust-to-weight"),
        ("drag_coefficients", [0.1, -0.1, 0.0], "drag"),
        ("unknown_key", 1, "Extra inputs are not permitted"),
    ],
)
def test_invalid_vehicle_values_are_rejected(key: str, value: Any, message: str) -> None:
    data = iris_dict()
    data[key] = value
    with pytest.raises(ConfigError, match=message):
        parse_config(data, VehicleConfig, "test")


def test_invalid_rotor_direction_rejected() -> None:
    data = iris_dict()
    data["rotors"][0]["direction"] = 0
    with pytest.raises(ConfigError, match="direction"):
        parse_config(data, VehicleConfig, "test")


def test_sim_rejects_bad_integrator_and_start_below_ground() -> None:
    with pytest.raises(ConfigError, match="integrator"):
        parse_config({"integrator": "verlet"}, PythonSimConfig)
    with pytest.raises(ConfigError, match="below the ground"):
        parse_config({"initial_state": {"position": [0, 0, -1]}}, PythonSimConfig)


def test_error_message_names_file_and_field(tmp_path: Path) -> None:
    data = {"vehicle": iris_dict()}
    data["vehicle"]["mass"] = 0.0
    file = tmp_path / "bad.yaml"
    file.write_text(yaml.safe_dump(data))
    with pytest.raises(ConfigError) as exc:
        load_config(file, VehicleConfig, section="vehicle")
    assert str(file) in str(exc.value) and "mass" in str(exc.value)


def test_loader_errors(tmp_path: Path) -> None:
    with pytest.raises(ConfigError, match="not found"):
        load_yaml("does_not_exist.yaml")
    bad = tmp_path / "list.yaml"
    bad.write_text("- 1\n- 2\n")
    with pytest.raises(ConfigError, match="mapping"):
        load_yaml(bad)
    with pytest.raises(ConfigError, match="missing top-level section"):
        load_config("drone_iris.yaml", VehicleConfig, section="nope")


def test_configs_are_immutable(iris: VehicleConfig) -> None:
    with pytest.raises(Exception, match="frozen"):
        iris.mass = 2.0  # type: ignore[misc]
