"""YAML loading with schema validation and readable error messages."""

from __future__ import annotations

from pathlib import Path
from typing import Any, TypeVar

import yaml
from pydantic import BaseModel, ValidationError

ModelT = TypeVar("ModelT", bound=BaseModel)

#: Repository root (``src/dronenav/config/loader.py`` -> three levels up).
REPO_ROOT = Path(__file__).resolve().parents[3]
#: Default directory holding the YAML configuration files.
CONFIG_DIR = REPO_ROOT / "config"


class ConfigError(ValueError):
    """Raised when a configuration file is missing, malformed or invalid."""


def resolve_config_path(path: str | Path) -> Path:
    """Resolve a config path: absolute, relative to CWD, or relative to ``config/``.

    Args:
        path: File path or bare file name (e.g. ``"drone_iris.yaml"``).

    Returns:
        The existing file path.

    Raises:
        ConfigError: If no matching file exists.
    """
    candidate = Path(path)
    for option in (candidate, CONFIG_DIR / candidate):
        if option.is_file():
            return option.resolve()
    raise ConfigError(f"configuration file not found: {path} (also looked in {CONFIG_DIR})")


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Read a YAML file that must contain a mapping at the top level.

    Raises:
        ConfigError: If the file is missing, is not valid YAML or is not a mapping.
    """
    file = resolve_config_path(path)
    try:
        with file.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ConfigError(f"{file}: invalid YAML: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"{file}: top-level element must be a mapping, got {type(data).__name__}")
    return data


def parse_config(data: dict[str, Any], model: type[ModelT], source: str = "<dict>") -> ModelT:
    """Validate a dictionary against a schema.

    Args:
        data: Raw configuration values.
        model: Pydantic model class to validate against.
        source: Description of the origin, used in error messages.

    Returns:
        The validated (immutable) model instance.

    Raises:
        ConfigError: With one line per invalid field.
    """
    try:
        return model.model_validate(data)
    except ValidationError as exc:
        lines = [f"{source}: invalid {model.__name__} ({exc.error_count()} error(s))"]
        for err in exc.errors():
            location = ".".join(str(part) for part in err["loc"]) or "<root>"
            lines.append(f"  - {location}: {err['msg']}")
        raise ConfigError("\n".join(lines)) from exc


def load_config(path: str | Path, model: type[ModelT], section: str | None = None) -> ModelT:
    """Load a YAML file (optionally one top-level section) and validate it.

    Args:
        path: YAML file path or name inside ``config/``.
        model: Pydantic model class.
        section: Optional top-level key to validate instead of the whole file.

    Returns:
        The validated model instance.

    Raises:
        ConfigError: If the file or the section is missing or invalid.
    """
    file = resolve_config_path(path)
    data = load_yaml(file)
    source = str(file)
    if section is not None:
        if section not in data:
            raise ConfigError(f"{file}: missing top-level section '{section}'")
        data = data[section]
        source = f"{file} [{section}]"
        if not isinstance(data, dict):
            raise ConfigError(f"{source}: section must be a mapping")
    return parse_config(data, model, source)
