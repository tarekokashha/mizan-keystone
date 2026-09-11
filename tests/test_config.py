"""Tests for keystone.config: UR5eConfig, its provenance machinery and content hash.

UR5eConfig subclasses lerobot.robots.RobotConfig (see docs/decisions/
task-1-2-correction.md). RobotConfig is a non-frozen dataclass and Python
forbids a frozen subclass of a non-frozen one, so immutability here is
enforced by a custom __setattr__, not by @dataclass(frozen=True). id and
calibration_dir are inherited from RobotConfig and must not be redeclared.
"""
from __future__ import annotations

import dataclasses

import pytest
from lerobot.robots import RobotConfig

from keystone.config import UR5eConfig


def _minimal_kwargs(**overrides) -> dict:
    """The smallest set of constructor kwargs that satisfies provenance.

    Deliberately does not pass id, so tests can check it stays whatever
    RobotConfig's own default is: proof that id is inherited, not redeclared
    with a different default.
    """
    kwargs = dict(
        ip="192.0.2.10",
        control_hz=125.0,
        servo_lookahead=0.1,
        servo_gain=300.0,
        gripper="placeholder",
        cameras={},
        provenance={
            "ip": "declared",
            "control_hz": "declared",
            "servo_lookahead": "declared",
            "servo_gain": "declared",
            "gripper": "declared",
            "cameras": "declared",
        },
    )
    kwargs.update(overrides)
    return kwargs


# ---- the corrected LeRobot contract -------------------------------------- #

def test_subclasses_lerobot_robot_config():
    assert issubclass(UR5eConfig, RobotConfig)


def test_registered_as_ur5e_choice():
    # This is what makes `--robot.type=ur5e` resolvable from the command
    # line: RobotConfig is a draccus.ChoiceRegistry and UR5eConfig must be
    # registered into it under the name "ur5e".
    assert RobotConfig.get_choice_class("ur5e") is UR5eConfig
    assert UR5eConfig.declared().type == "ur5e"


def test_id_and_calibration_dir_are_inherited_not_redeclared():
    # If UR5eConfig redeclared `id` with its own default, this constructed
    # instance (which does not pass id) would not show RobotConfig's None
    # default. It must, because id is inherited, not redeclared.
    cfg = UR5eConfig(**_minimal_kwargs())
    assert cfg.id is None
    assert cfg.calibration_dir is None
    own_field_names = {f.name for f in dataclasses.fields(UR5eConfig)}
    assert "id" in own_field_names  # inherited fields still appear here
    assert "calibration_dir" in own_field_names
    # The field objects themselves must come from RobotConfig, not be
    # shadowed by a field UR5eConfig declared with the same name.
    parent_id_field = next(f for f in dataclasses.fields(RobotConfig) if f.name == "id")
    own_id_field = next(f for f in dataclasses.fields(UR5eConfig) if f.name == "id")
    assert own_id_field.default == parent_id_field.default


def test_declared_factory_sets_an_id():
    assert UR5eConfig.declared().id == "ur5e"


# ---- the provenance rule --------------------------------------------------- #

def test_config_carries_no_safety_limits():
    # The skeleton had max_joint_velocity and max_force_norm here. They are
    # envelope values. Two sources of truth for a safety limit is worse than
    # one, and the Shield owns the envelope.
    cfg = UR5eConfig.declared()
    for forbidden in ("max_joint_velocity", "max_force_norm", "q_min", "q_max"):
        assert not hasattr(cfg, forbidden), (
            f"{forbidden} belongs to sentinel.Envelope, not to a driver config")


def test_measured_provenance_is_refused_without_a_named_human():
    d = UR5eConfig.declared().to_dict()
    d["provenance"]["control_hz"] = "measured"
    with pytest.raises(ValueError, match="measured"):
        UR5eConfig.from_dict(d)


def test_measured_provenance_is_refused_with_blank_human_or_date():
    # A whitespace string is truthy in Python. measured_by or measured_on set
    # to blanks must not pass a bare truthiness check.
    d = UR5eConfig.declared().to_dict()
    d["provenance"]["control_hz"] = "measured"
    d["measured_by"] = "   "
    d["measured_on"] = "2026-09-11"
    with pytest.raises(ValueError, match="measured"):
        UR5eConfig.from_dict(d)


def test_measured_provenance_is_accepted_with_a_named_human_and_date():
    # Exercises the acceptance path only: this is a synthetic value for the
    # test fixture, not a claim that any hardware was measured. No real UR5e
    # was run to produce 123.0.
    d = UR5eConfig.declared().to_dict()
    d["provenance"]["control_hz"] = "measured"
    d["control_hz"] = 123.0
    d["measured_by"] = "a-test-fixture"
    d["measured_on"] = "2026-01-01"
    cfg = UR5eConfig.from_dict(d)
    assert cfg.control_hz == 123.0
    assert cfg.provenance["control_hz"] == "measured"


def test_unknown_provenance_value_is_rejected():
    d = UR5eConfig.declared().to_dict()
    d["provenance"]["control_hz"] = "vibes"
    with pytest.raises(ValueError, match="provenance"):
        UR5eConfig.from_dict(d)


def test_missing_provenance_entry_is_rejected():
    d = UR5eConfig.declared().to_dict()
    del d["provenance"]["control_hz"]
    with pytest.raises(ValueError, match="provenance"):
        UR5eConfig.from_dict(d)


# ---- content hash ----------------------------------------------------------- #

def test_sha256_is_stable_and_sensitive():
    a = UR5eConfig.declared()
    assert a.sha256() == UR5eConfig.declared().sha256()
    assert a.replace(servo_gain=a.servo_gain + 1).sha256() != a.sha256()


def test_json_round_trip():
    a = UR5eConfig.declared()
    b = UR5eConfig.from_json(a.to_json())
    assert a.to_dict() == b.to_dict()
    assert a.sha256() == b.sha256()


# ---- immutability, the hard way ---------------------------------------------- #
# RobotConfig is a non-frozen dataclass and Python forbids a frozen subclass
# of a non-frozen one, so UR5eConfig enforces immutability through a custom
# __setattr__ rather than @dataclass(frozen=True). M-01 found frozen=True
# alone did not stop a mutable provenance dict or a writeable array from
# being changed after construction anyway, so the same defences are needed
# either way.

def test_provenance_is_read_only_after_construction():
    cfg = UR5eConfig.declared()
    with pytest.raises(TypeError):
        cfg.provenance["control_hz"] = "measured"


def test_cameras_mapping_is_read_only_after_construction():
    cfg = UR5eConfig.declared()
    with pytest.raises(TypeError):
        cfg.cameras["webcam"] = {"fps": 30}


def test_attribute_rebinding_after_construction_is_rejected():
    cfg = UR5eConfig.declared()
    with pytest.raises(AttributeError):
        cfg.control_hz = 999.0


def test_inherited_field_rebinding_after_construction_is_also_rejected():
    # The immutability defence must cover fields inherited from RobotConfig
    # too, not just the ones UR5eConfig declares itself.
    cfg = UR5eConfig.declared()
    with pytest.raises(AttributeError):
        cfg.id = "not-ur5e-anymore"


def test_replace_builds_a_new_instance_without_mutating_the_original():
    a = UR5eConfig.declared()
    b = a.replace(servo_gain=a.servo_gain + 1)
    assert a.servo_gain != b.servo_gain
    assert a.sha256() != b.sha256()
