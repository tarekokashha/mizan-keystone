# Tasks 1 and 2 correction: the real LeRobot contract, measured

The plan said Task 3's plugin contract was "the one thing most likely to go
wrong", because it depended on an installed library's abstract API that the plan
had not inspected. It has now been inspected, and it changes Task 1.

Everything below was obtained by executing against the installed `lerobot`
0.4.4, not by reading documentation.

## Finding 1: the kit's skeleton is not a LeRobot plugin

`lerobot.robots.Robot` declares **ten** abstract members:

```
action_features       property
calibrate             method
configure             method        <- the skeleton does not have this
connect               method
disconnect            method
get_observation       method
is_calibrated         property      <- the skeleton does not have this
is_connected          property
observation_features  property
send_action           method
```

The skeleton implements eight of them. Instantiating a subclass with exactly the
skeleton's method set raises:

```
TypeError: Can't instantiate abstract class Probe with abstract methods
           configure, is_calibrated
```

So the skeleton would fail at construction, before any RTDE call. The design
document said it "declares the right method names and satisfies no abstract
API"; that is now a measurement rather than an assertion.

`UR5eFollower` must implement all ten.

## Finding 2: `UR5eConfig` cannot be a frozen dataclass

The plan specified a frozen dataclass, mirroring M-01's `Envelope`. That is not
possible here.

```
lerobot.robots.RobotConfig
  is dataclass : True
  frozen       : False
  fields       : id (str | None = None), calibration_dir (Path | None = None)
  MRO          : RobotConfig, ChoiceRegistry, ChoiceRegistryBase, ChoiceType,
                 Protocol, Generic, ABC, object
```

`Robot.__init__` takes a `RobotConfig`, and `ChoiceRegistry` is the mechanism
behind `lerobot-record --robot.type=ur5e`, which the skeleton's header names as a
requirement. So `UR5eConfig` must subclass `RobotConfig`.

Python forbids a frozen dataclass inheriting from a non-frozen one, so
`@dataclass(frozen=True)` is unavailable.

### What to do instead

Subclass `RobotConfig` and get immutability the hard way, because the property
still matters: M-01 found that `frozen=True` alone was not enough anyway, since a
mutable `provenance` dict and writeable numpy arrays defeated it after
construction.

```python
@dataclass
class UR5eConfig(RobotConfig):
    """Connection and servo parameters for a UR5e follower.

    Not frozen, and not by choice. RobotConfig is a non-frozen dataclass and
    Python forbids a frozen subclass of one, but subclassing it is required:
    Robot.__init__ takes a RobotConfig, and its ChoiceRegistry base is what
    makes `--robot.type=ur5e` resolvable from the command line.

    Immutability is therefore enforced here rather than by the decorator. That
    is no weaker than the alternative: M-01 SENTINEL found frozen=True alone did
    not prevent a mutable provenance mapping or a writeable numpy array from
    being changed after construction, so the same defences are needed either way.
    """

    _frozen: bool = field(default=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        ...validate...
        object.__setattr__(self, "provenance", MappingProxyType(dict(self.provenance)))
        object.__setattr__(self, "_frozen", True)

    def __setattr__(self, name, value):
        if getattr(self, "_frozen", False):
            raise AttributeError(
                f"UR5eConfig is immutable after construction; use replace() to "
                f"derive a new one. Attempted to set {name!r}."
            )
        object.__setattr__(self, name, value)
```

Test that post-construction mutation raises, and that `provenance` cannot be
mutated through the returned mapping. Both are the defects M-01 shipped and had
to fix; do not re-ship them.

`replace()` builds a new instance rather than mutating, the same as M-01's.

## Finding 3: what `RobotConfig` already gives you

`id` and `calibration_dir` are inherited. Do not redeclare them. The skeleton had
its own `id` field; that becomes the inherited one.

## Unchanged

Everything else in Tasks 1 and 2 stands. In particular:

- `max_joint_velocity` and `max_force_norm` stay **absent**. They are envelope
  values owned by `sentinel.Envelope`, and the test asserting their absence is
  the point, not an oversight.
- The RTDE fake is unaffected: all 18 methods the driver needs were verified
  present on the real interfaces, and the fidelity check is still the most
  important step in Task 2.
