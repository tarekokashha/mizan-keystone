"""UR5e driver configuration: connection and servo parameters, not safety limits.

Mirrors sentinel.envelope's provenance machinery, because the same defeats
apply here. A review of M-01 found that `frozen=True` alone let both
provenance and array fields be changed after construction: freezing a
dataclass blocks rebinding an attribute, not mutating the mutable object an
attribute points at. So provenance and cameras are stored as read-only
views, recursively, because a shallow MappingProxyType still hands back
the caller's own nested dicts, and loading a "measured" value without a
named human and a date is refused.

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
construction is enforced by custom __setattr__ and __delattr__ methods
instead, both guarded by a _frozen sentinel set true at the end of
__post_init__. Both guards are needed for parity: frozen=True refuses
`del cfg.x` as well as `cfg.x = y`, and a guarded __setattr__ on its own
left `del cfg._frozen` as a two-token way to switch immutability off for
the whole object, because _frozen has a plain default and so survives as a
class attribute after the instance attribute is deleted, at which point
the guard reads False back off the class. With both in place this is no
weaker than frozen=True would have been. Parity is the claim, not
invulnerability: object.__setattr__ and writing straight into cfg.__dict__
defeat frozen=True too, and they defeat this the same way. Nor is parity
sufficient on its own, which is why it is not the whole story here: M-01
already found that frozen=True did not stop a mutable provenance dict or a
writeable array from being changed after construction, so the read-only
views below (MappingProxyType, applied recursively) are needed either way.

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


def _deep_freeze(value: object) -> object:
    """A read-only view of value, all the way down.

    MappingProxyType(dict(m)) is a shallow copy: it refuses
    `cfg.cameras[k] = v` but not `cfg.cameras[k]["fps"] = 1`, and the
    nested dict it hands back is still the caller's own object, so anyone
    holding that handle can change the config, and with it sha256(), after
    construction. cameras is a mapping of mappings, so the freeze has to
    reach the bottom. Lists become tuples for the same reason; _plain()
    turns them back into lists on the way out.
    """
    if isinstance(value, collections.abc.Mapping):
        return MappingProxyType({k: _deep_freeze(v) for k, v in value.items()})
    if isinstance(value, (list, tuple)):
        return tuple(_deep_freeze(v) for v in value)
    return value


def _plain(value: object) -> object:
    """The inverse of _deep_freeze: plain, JSON-serialisable, unaliased.

    Every mapping and sequence is rebuilt, so what to_dict() returns
    shares no mutable object with the config it came from.
    """
    if isinstance(value, collections.abc.Mapping):
        return {k: _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    return value


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

    def __delattr__(self, name: str) -> None:
        # The same guard as __setattr__, and not decoration: without it,
        # `del cfg._frozen` alone switches immutability off. _frozen has a
        # plain default, so deleting the instance attribute leaves the
        # class attribute False behind for the guard above to read, and
        # every field becomes rebindable again, including a servo_gain the
        # constructor refused. servo_gain is passed straight to servoJ and
        # the Shield never sees it, so that barrier has to hold.
        if getattr(self, "_frozen", False):
            raise AttributeError(
                f"UR5eConfig is immutable after construction; use replace() to "
                f"derive a new one. Attempted to delete {name!r}."
            )
        object.__delattr__(self, name)

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
        # "measured"`. A MappingProxyType view closes that, but only at the
        # level it is applied to: cameras is a mapping of mappings, and a
        # shallow proxy leaves `cfg.cameras["webcam"]["fps"] = 1` open, with
        # the caller's own nested dict still aliased into the config. So the
        # freeze is recursive.
        object.__setattr__(self, "provenance", _deep_freeze(self.provenance))
        object.__setattr__(self, "cameras", _deep_freeze(self.cameras))

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
                # _plain, not dict(v): a shallow copy hands the caller the
                # config's own nested mappings back, and writing into one of
                # those changes sha256() after construction.
                d[f.name] = _plain(v)
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
