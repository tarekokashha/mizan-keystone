"""UR5e driver configuration: connection and servo parameters, not safety limits.

Mirrors sentinel.envelope's provenance machinery, because the same defeats
apply here. A review of M-01 found that `frozen=True` alone let both
provenance and array fields be changed after construction: freezing a
dataclass blocks rebinding an attribute, not mutating the mutable object an
attribute points at. So provenance is stored as a MappingProxyType, and
loading a "measured" value without a named human and a date is refused.

What is deliberately absent matters as much as what is present. The kit's
skeleton driver config carried `max_joint_velocity` and `max_force_norm`, and
this one does not, on purpose: those are envelope values and belong to
sentinel.Envelope. Two sources of truth for a safety limit is worse than one.

UR5eConfig subclasses lerobot.robots.RobotConfig rather than standing alone.
That is required, not stylistic: Robot.__init__ takes a RobotConfig, and
RobotConfig's ChoiceRegistry base is what makes `--robot.type=ur5e`
resolvable from the LeRobot command line. RobotConfig is measured to be a
non-frozen dataclass (see docs/decisions/task-1-2-correction.md), and Python
forbids a frozen dataclass inheriting from a non-frozen one, so
@dataclass(frozen=True) is not available here. Immutability after
construction is enforced by a custom __setattr__ instead, guarded by a
_frozen sentinel set true at the end of __post_init__. This is no weaker than
frozen=True would have been: M-01 already found that frozen=True alone did
not stop a mutable provenance dict or a writeable array from being changed
after construction, so the same defences (MappingProxyType, read-only
arrays) are needed either way, and a custom __setattr__ closes the same gap
that frozen=True closes for plain attribute rebinding.

id and calibration_dir are inherited from RobotConfig and are deliberately
not redeclared here.
"""
from __future__ import annotations

import collections.abc
import dataclasses
import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType

from lerobot.robots import RobotConfig

PROVENANCE_VALUES = ("datasheet", "declared", "measured")

# Facts about the installed ur_rtde 1.6.5 and the UR e-series RTDE interface,
# verified against the library's own pybind11 docstrings (servoJ's parameter
# ranges) and Universal Robots' published Real-Time Data Exchange
# specification (the 500 Hz maximum control loop rate). These bound what is a
# valid argument to the RTDE interfaces; they are not a motion envelope, and
# they are not written into any field's provenance below. This mirrors
# sentinel.envelope's DATASHEET_* constants, which bound Envelope's fields
# the same way without being asserted as their provenance: the envelope
# itself is what the owner declares, inside those bounds.
DATASHEET_MAX_CONTROL_HZ = 500.0
DATASHEET_SERVO_LOOKAHEAD_RANGE = (0.03, 0.2)
DATASHEET_SERVO_GAIN_RANGE = (100.0, 2000.0)

# Fields that identify or annotate a config rather than describe the robot:
# no provenance entry is required for these. id and calibration_dir are
# inherited from RobotConfig, not declared here, but dataclasses.fields()
# still walks them, so they must be exempted the same as the fields this
# module does declare. Mirrors sentinel.envelope's _META_FIELDS.
_META_FIELDS = ("id", "calibration_dir", "provenance", "measured_by", "measured_on")


@RobotConfig.register_subclass("ur5e")
@dataclass(eq=False)
class UR5eConfig(RobotConfig):
    """Connection and servo parameters for the UR5e driver.

    No safety limit lives here. max_joint_velocity, max_force_norm, q_min and
    q_max belong to sentinel.Envelope and are deliberately absent: the Shield
    owns the envelope, and this config does not duplicate it.

    eq=False mirrors sentinel.envelope.Envelope: identity of two configs is
    decided by comparing content, via sha256() or to_dict(), not by dataclass
    field-by-field equality.

    Not frozen, and not by choice: see the module docstring.
    """

    ip: str
    control_hz: float
    servo_lookahead: float
    servo_gain: float
    gripper: str
    cameras: collections.abc.Mapping[str, dict]
    provenance: dict = field(default_factory=dict)
    measured_by: str | None = None
    measured_on: str | None = None
    # Set true at the end of __post_init__. Not part of the public contract:
    # excluded from repr, from equality (already off via eq=False on the
    # class), and from serialisation because to_dict() skips any field name
    # starting with "_".
    _frozen: bool = field(default=False, repr=False, compare=False)

    def __setattr__(self, name: str, value: object) -> None:
        if getattr(self, "_frozen", False):
            raise AttributeError(
                f"UR5eConfig is immutable after construction; use replace() to "
                f"derive a new one. Attempted to set {name!r}."
            )
        object.__setattr__(self, name, value)

    def __post_init__(self) -> None:
        # Deliberately does not call RobotConfig.__post_init__(): that method
        # assumes self.cameras holds lerobot CameraConfig objects and probes
        # config.width / config.height / config.fps on each entry. This
        # config's cameras field holds plain dicts (no camera support has
        # been implemented yet; see LIMITATIONS.md), so that probe would
        # raise AttributeError on any non-empty cameras mapping rather than
        # the ValueError RobotConfig intends. cameras stays empty in
        # declared() and this module validates it itself below.

        # Same defeat as sentinel.envelope.Envelope: a plain dict stored on
        # an otherwise-immutable object can still be mutated in place after
        # construction, because attribute-rebinding protection only stops
        # `cfg.provenance = other_dict`, not `cfg.provenance["x"] =
        # "measured"`. A MappingProxyType view closes that.
        object.__setattr__(self, "provenance", MappingProxyType(dict(self.provenance)))
        object.__setattr__(self, "cameras", MappingProxyType(dict(self.cameras)))

        if not self.ip:
            raise ValueError("ip must not be empty")
        if not (0.0 < self.control_hz <= DATASHEET_MAX_CONTROL_HZ):
            raise ValueError(
                f"control_hz must be in (0, {DATASHEET_MAX_CONTROL_HZ}], "
                "the UR e-series RTDE maximum control loop rate"
            )
        lo, hi = DATASHEET_SERVO_LOOKAHEAD_RANGE
        if not (lo <= self.servo_lookahead <= hi):
            raise ValueError(
                f"servo_lookahead must be in [{lo}, {hi}], the valid range for "
                "ur_rtde's servoJ lookahead_time"
            )
        lo, hi = DATASHEET_SERVO_GAIN_RANGE
        if not (lo <= self.servo_gain <= hi):
            raise ValueError(
                f"servo_gain must be in [{lo}, {hi}], the valid range for "
                "ur_rtde's servoJ gain"
            )
        if not self.gripper:
            raise ValueError("gripper must not be empty")

        bad = {k: v for k, v in self.provenance.items() if v not in PROVENANCE_VALUES}
        if bad:
            raise ValueError(f"unknown provenance values: {bad}")
        missing = [
            f.name for f in dataclasses.fields(self)
            if f.name not in _META_FIELDS
            and not f.name.startswith("_")
            and f.name not in self.provenance
        ]
        if missing:
            raise ValueError(f"these fields have no provenance: {missing}")

        # A whitespace string is truthy in Python, so measured_by="   " would
        # pass a bare truthiness check as though it named a human. Strip
        # before judging, mirroring the identical fix in
        # sentinel.envelope.Envelope.__post_init__.
        measured_by_named = bool(self.measured_by) and bool(self.measured_by.strip())
        measured_on_named = bool(self.measured_on) and bool(str(self.measured_on).strip())
        if "measured" in self.provenance.values() and not (measured_by_named and measured_on_named):
            raise ValueError(
                "a provenance of 'measured' requires measured_by and measured_on; "
                "only a human who ran the arm may write a measured number"
            )

        object.__setattr__(self, "_frozen", True)

    # ---- serialisation ---------------------------------------------------- #
    def to_dict(self) -> dict:
        d: dict = {}
        for f in dataclasses.fields(self):
            if f.name.startswith("_"):
                continue
            v = getattr(self, f.name)
            if isinstance(v, collections.abc.Mapping):
                d[f.name] = dict(v)
            elif isinstance(v, Path):
                d[f.name] = str(v)
            else:
                d[f.name] = v
        return d

    @classmethod
    def from_dict(cls, d: dict) -> UR5eConfig:
        return cls(**dict(d))

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    @classmethod
    def from_json(cls, s: str) -> UR5eConfig:
        return cls.from_dict(json.loads(s))

    def sha256(self) -> str:
        """Content hash of the configuration. Goes in every run journal line."""
        return hashlib.sha256(self.to_json().encode("utf-8")).hexdigest()

    def replace(self, **kw) -> UR5eConfig:
        return dataclasses.replace(self, **kw)

    # ---- factory ------------------------------------------------------------ #
    @classmethod
    def declared(cls) -> UR5eConfig:
        """The conservative default. Every field is declared, none measured.

        The IP address is a reserved documentation address (RFC 5737
        TEST-NET-1), not a real device's address, because this repository
        never writes a real-robot number.
        """
        return cls(
            ip="192.0.2.10",
            control_hz=125.0,
            servo_lookahead=0.1,
            servo_gain=300.0,
            gripper="placeholder",
            cameras={},
            id="ur5e",
            provenance={
                "ip": "declared",
                "control_hz": "declared",
                "servo_lookahead": "declared",
                "servo_gain": "declared",
                "gripper": "declared",
                "cameras": "declared",
            },
        )
