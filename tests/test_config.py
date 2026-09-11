"""Tests for keystone.config: UR5eConfig, its provenance machinery and content hash.

UR5eConfig subclasses lerobot.robots.RobotConfig (see docs/decisions/
task-1-2-correction.md). RobotConfig is a non-frozen dataclass and Python
forbids a frozen subclass of a non-frozen one, so immutability here is
enforced by custom __setattr__ and __delattr__ methods, not by
@dataclass(frozen=True). Both are tested, because a guard on rebinding
alone leaves deletion open and deleting the freeze sentinel switches the
rest off. id and calibration_dir are inherited from RobotConfig and must
not be redeclared.
"""
from __future__ import annotations

import dataclasses
import ipaddress
import re
from pathlib import Path

import pytest
from lerobot.robots import RobotConfig

from keystone.config import (
    DATASHEET_MAX_CONTROL_HZ,
    DATASHEET_SERVO_GAIN_RANGE,
    DATASHEET_SERVO_LOOKAHEAD_RANGE,
    UR5eConfig,
    _META_FIELDS,
)


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


# ---- the datasheet ranges: the only barrier between a caller and servoJ ---- #
# The Shield authorises q and nothing else. servo_lookahead and servo_gain are
# never shown to it: they go straight into ur_rtde's servoJ as its
# lookahead_time and gain arguments, and control_hz sets the rate the RTDE
# interface is driven at. __post_init__ is the whole barrier between a
# caller-supplied number and the controller, so it is tested here as one.
# Out-of-range values below are written as literals on purpose, not derived
# from the DATASHEET_* constants: a test that says "one past the constant" is
# satisfied by any constant, including a widened one.

@pytest.mark.parametrize("bad", [0.0, -1.0, -125.0, 500.1, 1e9])
def test_control_hz_outside_the_rtde_range_is_rejected(bad):
    with pytest.raises(ValueError, match="control_hz"):
        UR5eConfig(**_minimal_kwargs(control_hz=bad))


@pytest.mark.parametrize("ok", [0.001, 125.0, 500.0])
def test_control_hz_inside_the_rtde_range_is_accepted(ok):
    # The upper boundary is inclusive and must stay that way: this is what
    # stops a failing rejection test from being "fixed" by narrowing the check.
    assert UR5eConfig(**_minimal_kwargs(control_hz=ok)).control_hz == ok


@pytest.mark.parametrize("bad", [0.0, -0.1, 0.029, 0.201, 1.0, 1e9])
def test_servo_lookahead_outside_the_servoj_range_is_rejected(bad):
    with pytest.raises(ValueError, match="servo_lookahead"):
        UR5eConfig(**_minimal_kwargs(servo_lookahead=bad))


@pytest.mark.parametrize("ok", [0.03, 0.1, 0.2])
def test_servo_lookahead_inside_the_servoj_range_is_accepted(ok):
    assert UR5eConfig(**_minimal_kwargs(servo_lookahead=ok)).servo_lookahead == ok


@pytest.mark.parametrize("bad", [0.0, -1.0, 99.9, 2000.1, 1e6, 1e9])
def test_servo_gain_outside_the_servoj_range_is_rejected(bad):
    with pytest.raises(ValueError, match="servo_gain"):
        UR5eConfig(**_minimal_kwargs(servo_gain=bad))


@pytest.mark.parametrize("ok", [100.0, 300.0, 2000.0])
def test_servo_gain_inside_the_servoj_range_is_accepted(ok):
    assert UR5eConfig(**_minimal_kwargs(servo_gain=ok)).servo_gain == ok


def test_datasheet_constants_are_the_installed_ur_rtde_own_ranges():
    """The ranges belong to the library, so read them back out of it.

    A range check is only worth what its bounds are worth. These two are
    documented by the installed ur_rtde in servoJ's own pybind11 docstring,
    and this test fails if the constants here ever drift from it.
    """
    import rtde_control

    doc = rtde_control.RTDEControlInterface.servoJ.__doc__
    found = [
        (float(lo), float(hi))
        for lo, hi in re.findall(r"range \[([-\d.]+),\s*([-\d.]+)\]", doc)
    ]
    assert len(found) == 2, f"servoJ docstring no longer states two ranges: {doc}"
    lookahead_range, gain_range = found
    assert DATASHEET_SERVO_LOOKAHEAD_RANGE == lookahead_range
    assert DATASHEET_SERVO_GAIN_RANGE == gain_range
    # 500 Hz is the UR e-series RTDE maximum control loop rate from Universal
    # Robots published RTDE specification, not from any docstring, so it is
    # pinned as a literal rather than read back from the library.
    assert DATASHEET_MAX_CONTROL_HZ == 500.0


@pytest.mark.parametrize("bad_ip", ["", None])
def test_empty_ip_is_rejected(bad_ip):
    with pytest.raises(ValueError, match="ip must not be empty"):
        UR5eConfig(**_minimal_kwargs(ip=bad_ip))


@pytest.mark.parametrize("bad_gripper", ["", None])
def test_empty_gripper_is_rejected(bad_gripper):
    with pytest.raises(ValueError, match="gripper must not be empty"):
        UR5eConfig(**_minimal_kwargs(gripper=bad_gripper))


# ---- immutability: deletion, not just rebinding ---------------------------- #

def test_attribute_deletion_after_construction_is_rejected():
    cfg = UR5eConfig.declared()
    with pytest.raises(AttributeError):
        del cfg.control_hz
    assert cfg.control_hz == UR5eConfig.declared().control_hz


def test_inherited_field_deletion_after_construction_is_also_rejected():
    cfg = UR5eConfig.declared()
    with pytest.raises(AttributeError):
        del cfg.id
    assert cfg.id == "ur5e"


def test_the_frozen_sentinel_itself_cannot_be_deleted():
    # _frozen has a plain default, so it exists as a class attribute as well
    # as an instance one. Deleting the instance attribute would leave the
    # guard's getattr(self, "_frozen", False) reading False off the class,
    # and two tokens would switch immutability off for the whole object.
    cfg = UR5eConfig.declared()
    with pytest.raises(AttributeError):
        del cfg._frozen
    assert cfg._frozen is True
    with pytest.raises(AttributeError):
        cfg.control_hz = 999.0


def test_a_servo_gain_the_constructor_refuses_stays_unreachable():
    # servo_gain reaches servoJ as its gain argument and the Shield never
    # sees it, so __post_init__ is the only barrier. Deleting the freeze
    # sentinel must not be a way around that barrier.
    out_of_range = 1e9
    with pytest.raises(ValueError, match="servo_gain"):
        UR5eConfig.declared().replace(servo_gain=out_of_range)
    cfg = UR5eConfig.declared()
    for attack in (
        lambda: delattr(cfg, "_frozen"),
        lambda: setattr(cfg, "servo_gain", out_of_range),
        lambda: delattr(cfg, "servo_gain"),
    ):
        with pytest.raises(AttributeError):
            attack()
    assert cfg.servo_gain == UR5eConfig.declared().servo_gain


def test_immutability_is_no_weaker_than_frozen_true():
    """Hold the module docstring's parity claim to an actual frozen dataclass.

    Whatever a frozen=True dataclass refuses, UR5eConfig must refuse too.
    dataclasses.FrozenInstanceError subclasses AttributeError, so one
    expectation covers both. Parity is the claim and not invulnerability:
    writing straight into cfg.__dict__ defeats frozen=True as well, so it is
    deliberately not asserted against here.
    """
    assert issubclass(dataclasses.FrozenInstanceError, AttributeError)

    @dataclasses.dataclass(frozen=True)
    class Control:
        servo_gain: float = 300.0

    control = Control()
    cfg = UR5eConfig.declared()
    operations = (
        lambda obj: setattr(obj, "servo_gain", 1e9),
        lambda obj: delattr(obj, "servo_gain"),
    )
    for operation in operations:
        with pytest.raises(AttributeError):
            operation(control)
        with pytest.raises(AttributeError):
            operation(cfg)


# ---- nested state: the same defect as M-01, one level deeper --------------- #
# sha256() is documented as the value that goes in every run journal line. A
# caller who still holds a handle to something inside the config must not be
# able to change that value after construction.

def test_nested_camera_config_is_read_only_after_construction():
    cfg = UR5eConfig(**_minimal_kwargs(cameras={"webcam": {"fps": 30}}))
    with pytest.raises(TypeError):
        cfg.cameras["webcam"]["fps"] = 1
    assert cfg.cameras["webcam"]["fps"] == 30


def test_mutating_the_cameras_dict_given_to_the_constructor_does_not_reach_it():
    inner = {"fps": 30}
    cfg = UR5eConfig(**_minimal_kwargs(cameras={"webcam": inner}))
    before = cfg.sha256()
    inner["fps"] = 1
    assert cfg.cameras["webcam"]["fps"] == 30
    assert cfg.sha256() == before


def test_mutating_the_result_of_to_dict_does_not_reach_the_config():
    cfg = UR5eConfig(**_minimal_kwargs(cameras={"webcam": {"fps": 30}}))
    before = cfg.sha256()
    d = cfg.to_dict()
    d["cameras"]["webcam"]["fps"] = 1
    d["provenance"]["ip"] = "measured"
    assert cfg.cameras["webcam"]["fps"] == 30
    assert cfg.provenance["ip"] == "declared"
    assert cfg.sha256() == before


def test_json_round_trip_survives_a_nested_camera_config():
    # Freezing the nested state must not cost serialisation fidelity: what
    # comes back out of to_dict() has to be plain, JSON-serialisable and
    # equal to what went in, lists included.
    cfg = UR5eConfig(**_minimal_kwargs(
        cameras={"webcam": {"fps": 30, "resolution": [640, 480]}}))
    back = UR5eConfig.from_json(cfg.to_json())
    assert back.to_dict() == cfg.to_dict()
    assert back.sha256() == cfg.sha256()
    assert cfg.to_dict()["cameras"] == {"webcam": {"fps": 30, "resolution": [640, 480]}}


# ---- what the hash covers, and what the factory declares ------------------- #

def test_sha256_is_sensitive_to_every_field():
    # The older sensitivity test varied one field. A hash that ignores a
    # field is a journal line that cannot tell two different runs apart, so
    # every public field gets a case, and a newly added field with no case
    # fails this test rather than slipping through unhashed.
    base = UR5eConfig.declared()
    variants = {
        "ip": "192.0.2.11",
        "control_hz": base.control_hz + 1.0,
        "servo_lookahead": base.servo_lookahead + 0.01,
        "servo_gain": base.servo_gain + 1.0,
        "gripper": "a-different-placeholder",
        "cameras": {"webcam": {"fps": 30}},
        "provenance": dict(base.provenance, ip="datasheet"),
        "measured_by": "a-test-fixture",
        "measured_on": "2026-01-01",
        "id": "ur5e-the-second",
        "calibration_dir": Path("calibration"),
    }
    public = {
        f.name for f in dataclasses.fields(UR5eConfig) if not f.name.startswith("_")
    }
    assert set(variants) == public, "a new field needs a hash sensitivity case"
    for name, value in variants.items():
        assert base.replace(**{name: value}).sha256() != base.sha256(), (
            f"sha256() ignores {name}")


def test_to_dict_carries_every_public_field():
    cfg = UR5eConfig.declared().replace(
        measured_by="a-test-fixture", measured_on="2026-01-01")
    d = cfg.to_dict()
    public = {
        f.name for f in dataclasses.fields(UR5eConfig) if not f.name.startswith("_")
    }
    assert set(d) == public
    assert d["measured_by"] == "a-test-fixture"
    assert d["measured_on"] == "2026-01-01"
    assert "_frozen" not in d


def test_declared_ip_is_a_documentation_address_not_a_real_robot():
    # declared()'s docstring says RFC 5737 TEST-NET-1 because this repository
    # never writes a real-robot number. Enforce the claim instead of trusting
    # the prose: a routable or site-local address must fail here.
    ip = ipaddress.ip_address(UR5eConfig.declared().ip)
    assert ip in ipaddress.ip_network("192.0.2.0/24"), (
        f"{ip} is not in RFC 5737 TEST-NET-1")
    assert not ip.is_global


def test_declared_factory_declares_every_field_and_measures_nothing():
    # "declared" against "datasheet" is a claim about where a number came
    # from. declared() ran no arm and read no datasheet for these values, so
    # every entry it writes is "declared", and only a human may write
    # "measured".
    cfg = UR5eConfig.declared()
    describing = {
        f.name for f in dataclasses.fields(UR5eConfig)
        if not f.name.startswith("_") and f.name not in _META_FIELDS
    }
    assert set(cfg.provenance) == describing
    assert set(cfg.provenance.values()) == {"declared"}
    assert cfg.measured_by is None
    assert cfg.measured_on is None
