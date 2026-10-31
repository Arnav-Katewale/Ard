import math

import pytest

from robodog.config import ConfigError, build, load_config, section
from robodog.servos.joint import (
    EmergencyStopActive,
    JointConfig,
    JointLimitError,
    SafeJointGroup,
    ServoBusConfig,
)
from robodog.servos.lx16a import Lx16aBus
from robodog.servos.simulated_servo import SimulatedLx16aPort

BUS_CONFIG = ServoBusConfig(device="sim", echo=False, max_joint_speed_dps=100.0, min_move_time_ms=20)
KNEE = JointConfig("FL_knee", servo_id=3, min_deg=20.0, max_deg=140.0, servo_neutral_deg=60.0, direction=1)
MIRRORED = JointConfig("FR_knee", servo_id=6, min_deg=20.0, max_deg=140.0, servo_neutral_deg=180.0, direction=-1)


def make_group(*joints):
    joints = joints or (KNEE,)
    port = SimulatedLx16aPort.with_servos([j.servo_id for j in joints])
    for j in joints:  # start each servo at joint angle 60 deg
        port.servos[j.servo_id].position_units = round(j.joint_to_servo_deg(60.0) / 0.24)
    group = SafeJointGroup(Lx16aBus(port), list(joints), BUS_CONFIG)
    group.sync_from_servos()
    return group, port


# --- joint <-> servo mapping --------------------------------------------------------


def test_direction_mapping_round_trip():
    assert KNEE.joint_to_servo_deg(30.0) == 90.0
    assert MIRRORED.joint_to_servo_deg(30.0) == 150.0
    assert MIRRORED.servo_to_joint_deg(150.0) == pytest.approx(30.0)


def test_limits_mapping_outside_servo_range_rejected():
    with pytest.raises(ConfigError, match="outside 0..240"):
        JointConfig("bad", servo_id=1, min_deg=0.0, max_deg=200.0, servo_neutral_deg=120.0, direction=1)


def test_duplicate_servo_ids_rejected():
    clash = JointConfig("other", servo_id=3, min_deg=0.0, max_deg=10.0, servo_neutral_deg=120.0, direction=1)
    with pytest.raises(ConfigError, match="duplicate servo ids"):
        SafeJointGroup(Lx16aBus(SimulatedLx16aPort()), [KNEE, clash], BUS_CONFIG)


def test_repository_joint_config_builds():
    cfg = load_config()
    joints = [build(JointConfig, j, "joints") for j in cfg["joints"]]
    build(ServoBusConfig, section(cfg, "servo_bus"), "servo_bus")
    assert len(joints) == 12


# --- command safety ---------------------------------------------------------------


def test_sync_reads_actual_position():
    group, _ = make_group(KNEE, MIRRORED)
    assert group.current_deg["FL_knee"] == pytest.approx(60.0, abs=0.25)
    assert group.current_deg["FR_knee"] == pytest.approx(60.0, abs=0.25)


def test_valid_command_moves_servo():
    group, port = make_group()
    group.command({"FL_knee": 65.0})
    assert port.servos[3].position_units * 0.24 == pytest.approx(KNEE.joint_to_servo_deg(65.0), abs=0.25)


@pytest.mark.parametrize("bad", [10.0, 150.0, math.nan])
def test_out_of_limit_target_rejected_not_clamped(bad):
    group, port = make_group()
    sent_before = len(port.written)
    with pytest.raises(JointLimitError):
        group.command({"FL_knee": bad})
    assert len(port.written) == sent_before  # nothing was sent


def test_one_bad_target_blocks_whole_command():
    group, port = make_group(KNEE, MIRRORED)
    sent_before = len(port.written)
    with pytest.raises(JointLimitError, match="FR_knee"):
        group.command({"FL_knee": 70.0, "FR_knee": 500.0})
    assert len(port.written) == sent_before


def test_unknown_joint_rejected():
    group, _ = make_group()
    with pytest.raises(JointLimitError, match="unknown joint"):
        group.command({"tail": 0.0})


def test_command_requires_known_start_position():
    port = SimulatedLx16aPort.with_servos([3])
    group = SafeJointGroup(Lx16aBus(port), [KNEE], BUS_CONFIG)
    with pytest.raises(JointLimitError, match="sync_from_servos"):
        group.command({"FL_knee": 60.0})


def test_move_time_respects_speed_limit():
    group, _ = make_group()
    # 50 deg at 100 deg/s -> 500 ms
    assert group.command({"FL_knee": 110.0}) == 500
    # tiny moves use the minimum move time
    assert group.command({"FL_knee": 110.5}) == 20


# --- emergency stop -----------------------------------------------------------------


def test_emergency_stop_halts_all_and_latches():
    group, port = make_group(KNEE, MIRRORED)
    assert group.emergency_stop("test") == []
    assert port.servos[3].stopped and port.servos[6].stopped
    with pytest.raises(EmergencyStopActive):
        group.command({"FL_knee": 60.0})
    group.reset_emergency_stop()
    group.command({"FL_knee": 60.0})


def test_emergency_stop_continues_past_unresponsive_servo():
    class FlakyBus:
        def stop(self, servo_id):
            if servo_id == 3:
                raise OSError("bus error")
            self.stopped = servo_id

    group = SafeJointGroup(FlakyBus(), [KNEE, MIRRORED], BUS_CONFIG)
    assert group.emergency_stop("test") == ["FL_knee"]
    assert group.estop_active
