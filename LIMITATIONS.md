# LIMITATIONS

What this repository does not establish. Read this before citing anything in
[README.md](README.md).

A safety repository that oversells itself is worse than one that does
nothing, because it spends credibility it has not earned and someone
downstream relies on it.

## The big one: no hardware, ever

Nothing here has touched a UR5e. Every number in this repository is either
software timing measured on a laptop or a trajectory produced by a simulated
plant. **A green suite here does not mean the arm is safe.** It means that no
action from a catalogue of fifteen attacks got past the kernel in simulation,
against a plant model that is not the arm.

The MIZAN operating contract's gate still stands: never execute an
adversarial instruction outside URSim until the controller enforces joint,
velocity and force envelopes in hardware. This programme does not lift that
gate. It builds the attachment and proves the attachment.

## Claim (B) is half unmeasured

End-to-end RTDE timing was never measured, because URSim could not be
started. On the host used, Docker Desktop's processes were running while
`docker version`, `docker ps` and `docker info` each returned no output, no
error, and had to be killed by a timeout.

The suite skips that integration with a reason rather than failing, which is
the designed behaviour, but a skip is not a pass. The half of claim (B) that
requires a controller is **unmeasured**, not "assumed fine".

Consequences worth stating separately:

- The timing figures in [PROTOCOL.md](PROTOCOL.md) section 9 are the cost of
  the **software** against a deterministic fake. The real figure including a
  socket, a NAT and a controller can only be worse, never better.
- The `ursim` CI job has **never been observed to pass**. It is marked
  UNVERIFIED in the workflow file itself. It exists so the path is there and
  is honest about its status, not because it is known to work.

## This platform offers no real-time guarantee, and the measurement says so

Max cycle time was 127 to 161 ms across all six measured runs. It appears in
the bare-driver runs as readily as in the composed ones, so it is the
operating system descheduling the process rather than anything the safety
layer does. On a 2 ms control period that is an outlier of roughly eighty
periods.

The composed p99.9 of 3457 to 3875 us also exceeds a 500 Hz deadline. The
vendor kit's gate of "p99.9 jitter < 2500 us" fails here in all three
repeats. That gate was never measured by its author and this programme does
not adopt it, relax it, or argue with it. It reports the measurement.

**Do not claim a 500 Hz hard deadline on a Windows host with no real-time
scheduling.** This measurement is the evidence for not claiming it.

## The envelope values are declared, not measured

Every limit the kernel enforces is owned by `sentinel.Envelope` and carries
provenance of `declared`. None of them is a measured property of a physical
arm. Only a human may mark a value `measured`, and none has been.

So the kernel provably holds the arm inside *a* box. Whether that box is the
right box for a real UR5e is a separate question this repository does not
answer.

## One declared guard is never exercised through the composition

`force_max` does not fire in any of the fifteen attack episodes, and the test
suite **asserts the reason rather than asserting the guard**:
`FakeReceive.getActualTCPForce()` returns a zero wrench because no contact
physics is modelled, so the guard has nothing to measure.

That is an honest exemption rather than a silent one, but it means the force
guard's behaviour in composition is untested. Closing it needs a contact
model in the fake, URSim, or an arm.

## The gripper is a placeholder and the authorisation path dead-ends

`config.gripper` is `"placeholder"`. The Shield authorises a
`gripper_position` and `UR5eFollower` silently drops it, because no gripper
hardware is driven. The `gripper_crush` attack's `grip_rate` verdict
therefore has no actuation path to protect. Consistent with the declared
config, and still a gap in what the composition demonstrates.

## The fake's fidelity is checked in two dimensions, not three

`keystone/rtde_fake.py` is load bearing: it is what makes claim (A)
exhaustive and offline, so a fake more permissive than the real interface
would turn every green test into a false negative.

What is checked, against the installed `ur_rtde` 1.6.5, for all eighteen
methods the driver uses:

- **Arity**, parsed from the real docstrings, because `ur_rtde` is a
  pybind11 extension and `inspect.signature` does not work on it. The check
  fails loudly if `inspect.signature` ever starts working, rather than going
  quiet.
- **Declared return type**, read from the first docstring line.

What is **not** checked: the real interface's runtime argument rejection. Its
constructors connect to hardware, so the C++ validators are unreachable
offline. No divergence is claimed there, because none could be measured. The
design specification's wording that the fake "must reject calls the real
interface rejects" is therefore only partly backed by evidence.

Two behaviours of the fake are pinned as deliberate decisions rather than
fixed:

- `servoJ` records the call **before** validating it, so a malformed command
  cannot skip the evidence base that claim (A) is read from.
- A NaN command puts the simulated state into an absorbing NaN, and
  `servoStop` cannot clear it. This is pinned by a test, not repaired.

Note also that the fake deliberately accepts NaN and inf. That is required,
not a gap: the architecture depends on the fake being a faithful dumb
transport so that the driver's lack of safety logic is observable. Adding
validation there would invalidate claim (A)'s tests.

## Immutability guards accidents, not sabotage

`UR5eConfig` reaches immutability through `__setattr__` and `__delattr__`
guards rather than `frozen=True`, because `RobotConfig` is a non-frozen
dataclass and Python forbids a frozen subclass of one. The claim made is
**parity with `frozen=True`**, and that claim is tested against a real frozen
dataclass in both directions.

Parity is not invulnerability. `object.__setattr__` and writing directly into
`cfg.__dict__` defeat `frozen=True` too, and defeat this the same way.
Likewise `FakeControl.calls` hands out records that refuse in-place mutation,
which guards the accident and not a determined caller.

## Claim (A) is exhaustive over a catalogue, not over all inputs

Claim (A) is tested by driving all fifteen attacks for 400 control steps
each, and `slow_drift` for 2000, giving 7600 commands. Every one was
bit-identical to what the kernel emitted.

That is a strong result and it is not a proof. It is exhaustive over
SENTINEL's attack catalogue at those horizons, not over all possible action
sequences. An attack nobody wrote is an attack nobody ran.

## The Shield defect is worked around here, not fixed upstream

M-03 found that `sentinel.shield.Shield` is not a `lerobot.robots.Robot`
subclass and implements eight of its ten abstract members. This repository
closes that with the `GuardedUR5e` adapter.

**The upstream defect still exists.** M-01 SENTINEL continues to ship a
Shield that cannot be handed to `lerobot-record` on its own. Anyone using
`sentinel` without `keystone` will hit it. It is recorded in
[docs/decisions/task-4-shield-is-not-a-lerobot-plugin.md](docs/decisions/task-4-shield-is-not-a-lerobot-plugin.md)
and belongs in M-01's own limitations.

## The LeRobot contract can move

`lerobot` is pinned at 0.4.4 and the plugin is written against the installed
base class, with a test that fails if the installed version's abstract API
changes. That converts a silent break into a loud one. It does not make the
plugin correct against any other version.

## What was found by review, and what that implies

An adversarial review of this repository mutated its own source and found
that four of the six defects this programme exists to remove could be
reintroduced into `send_action` with the entire suite green, and that the
config's range validation could be deleted entirely the same way. All of
those gaps are now closed and each fix is pinned by a test observed to fail
against the mutation that motivated it.

The implication is worth keeping: at that point the implementation was
correct and the suite was not, and only mutation testing could tell the
difference. The same is likely still true somewhere in this repository. The
findings above are the ones that were looked for and found, not a proof that
no others exist.
