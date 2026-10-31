"""Load robot configuration from a TOML file into validated dataclasses.

All tunable numbers live in config/robot.toml rather than in code. Each module
defines a small frozen dataclass for its own settings; `build()` turns a TOML
table into that dataclass and reports missing or unexpected keys clearly.
"""

from __future__ import annotations

import dataclasses
import tomllib
from pathlib import Path
from typing import Any, TypeVar

DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent.parent / "config" / "robot.toml"

T = TypeVar("T")


class ConfigError(ValueError):
    """Raised when the configuration is missing, malformed or unsafe."""


def load_config(path: str | Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    path = Path(path)
    if not path.is_file():
        raise ConfigError(f"Config file not found: {path}")
    try:
        with path.open("rb") as f:
            return tomllib.load(f)
    except tomllib.TOMLDecodeError as exc:
        raise ConfigError(f"Invalid TOML in {path}: {exc}") from exc


def section(config: dict[str, Any], dotted_name: str) -> dict[str, Any]:
    """Return a (possibly nested) table, e.g. section(cfg, "control.stability")."""
    node: Any = config
    for part in dotted_name.split("."):
        if not isinstance(node, dict) or part not in node:
            raise ConfigError(f"Missing config section [{dotted_name}]")
        node = node[part]
    if not isinstance(node, dict):
        raise ConfigError(f"Config entry [{dotted_name}] must be a table")
    return node


def build(cls: type[T], data: dict[str, Any], where: str) -> T:
    """Construct dataclass `cls` from `data`, rejecting missing and unknown keys.

    Validation of the values themselves is done in each dataclass's
    __post_init__, which should raise ConfigError.
    """
    fields = {f.name for f in dataclasses.fields(cls)}  # type: ignore[arg-type]
    required = {
        f.name
        for f in dataclasses.fields(cls)  # type: ignore[arg-type]
        if f.default is dataclasses.MISSING and f.default_factory is dataclasses.MISSING
    }
    missing = required - data.keys()
    unknown = data.keys() - fields
    problems = []
    if missing:
        problems.append(f"missing {sorted(missing)}")
    if unknown:
        problems.append(f"unknown {sorted(unknown)}")
    if problems:
        raise ConfigError(f"[{where}]: " + "; ".join(problems))
    return cls(**data)


def check(condition: bool, message: str) -> None:
    """Small helper for __post_init__ validation."""
    if not condition:
        raise ConfigError(message)
