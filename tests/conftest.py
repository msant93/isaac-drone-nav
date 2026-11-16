"""Shared pytest fixtures."""

from __future__ import annotations

import numpy as np
import pytest

from dronenav.config.loader import load_config
from dronenav.config.schemas import PythonSimConfig, VehicleConfig
from dronenav.model.quadrotor import QuadrotorModel
from dronenav.sim.python_sim import PythonSimBackend


@pytest.fixture(scope="session")
def iris() -> VehicleConfig:
    return load_config("drone_iris.yaml", VehicleConfig, section="vehicle")


@pytest.fixture(scope="session")
def sim_config() -> PythonSimConfig:
    return load_config("python_sim.yaml", PythonSimConfig, section="simulation")


@pytest.fixture()
def model(iris: VehicleConfig) -> QuadrotorModel:
    return QuadrotorModel(iris)


@pytest.fixture()
def sim(iris: VehicleConfig, sim_config: PythonSimConfig) -> PythonSimBackend:
    return PythonSimBackend(iris, sim_config)


@pytest.fixture()
def rng() -> np.random.Generator:
    return np.random.default_rng(12345)
