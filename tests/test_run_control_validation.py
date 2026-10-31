import csv
import importlib.util
import sys
from pathlib import Path

TOOL = Path(__file__).resolve().parent.parent / "tools" / "run_control_validation.py"


def load_tool():
    spec = importlib.util.spec_from_file_location("run_control_validation", TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_runner_prints_status_and_writes_csv(tmp_path, monkeypatch, capsys):
    out_csv = tmp_path / "run.csv"
    monkeypatch.setattr(sys, "argv", ["run_control_validation.py", "--duration", "1", "--every", "0.5",
                                      "--csv", str(out_csv)])
    assert load_tool().main() == 0

    text = capsys.readouterr().out
    assert "SIMULATED" in text and "Hardware integration testing is pending" in text
    for expected in ("Pitch:", "Roll:", "Obstacle:", "Stride length:", "Body roll correction:", "FL:", "RR:"):
        assert expected in text

    rows = list(csv.DictReader(out_csv.open(encoding="utf-8")))
    assert len(rows) == 50
    assert rows[0]["FL_knee"] != ""
