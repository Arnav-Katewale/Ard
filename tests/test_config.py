from dataclasses import dataclass

import pytest

from robodog.config import (
    DEFAULT_CONFIG_PATH,
    ConfigError,
    build,
    check,
    load_config,
    section,
)


@dataclass(frozen=True)
class RangeConfig:
    minimum: float
    maximum: float
    label: str = "range"

    def __post_init__(self):
        check(self.minimum < self.maximum, "minimum must be less than maximum")


def test_default_config_file_parses():
    assert isinstance(load_config(DEFAULT_CONFIG_PATH), dict)


def test_missing_file_raises(tmp_path):
    with pytest.raises(ConfigError, match="not found"):
        load_config(tmp_path / "nope.toml")


def test_invalid_toml_raises(tmp_path):
    bad = tmp_path / "bad.toml"
    bad.write_text("this is = = not toml")
    with pytest.raises(ConfigError, match="Invalid TOML"):
        load_config(bad)


def test_nested_section_lookup():
    cfg = {"control": {"stability": {"window_s": 2.0}}}
    assert section(cfg, "control.stability") == {"window_s": 2.0}
    with pytest.raises(ConfigError, match="control.gait"):
        section(cfg, "control.gait")


def test_build_valid():
    r = build(RangeConfig, {"minimum": 1, "maximum": 2}, "r")
    assert r.maximum == 2 and r.label == "range"


def test_build_reports_missing_and_unknown_keys():
    with pytest.raises(ConfigError) as exc:
        build(RangeConfig, {"minimum": 1, "maxx": 2}, "r")
    assert "missing ['maximum']" in str(exc.value)
    assert "unknown ['maxx']" in str(exc.value)


def test_build_runs_value_validation():
    with pytest.raises(ConfigError, match="minimum must be less"):
        build(RangeConfig, {"minimum": 5, "maximum": 2}, "r")
