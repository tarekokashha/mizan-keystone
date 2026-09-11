# Task 4 correction: the Shield is not a LeRobot plugin either

The plan gives Task 4 the interface
`guarded_ur5e(config, envelope=None, rtde=None) -> Shield`.

That return type does not work, for the same reason the vendor skeleton did
not work. This was measured against the installed `lerobot` 0.4.4 and
`mizan-sentinel`, not read.

## The finding

```
Shield subclasses lerobot Robot: False
Shield MRO: ['Shield', 'object']
Robot requires : action_features, calibrate, configure, connect, disconnect,
                 get_observation, is_calibrated, is_connected,
                 observation_features, send_action
Shield missing : configure, is_calibrated
```

`sentinel.shield.Shield` implements eight of the ten abstract members and
does not inherit from `lerobot.robots.Robot`. The two it lacks, `configure`
and `is_calibrated`, are **exactly** the two the vendor skeleton lacked.
See `docs/decisions/task-1-2-correction.md`, Finding 1, and the design
document's opening claim that the skeleton "declares the right method names
and satisfies no abstract API".

So M-01 SENTINEL reproduced, in the safety layer, the precise defect M-03
was chartered to fix in the driver layer. A `Shield` cannot be handed to
`lerobot-record`, and `isinstance(shield, Robot)` is `False`.

## Why M-01 did not catch it

Because it could not. M-01's stated largest limitation says every Shield
test runs against a fake implementing the LeRobot dict contract, since the
real driver lived in a repository M-01 did not touch. A fake built to the
dict contract cannot fail the abstract-API check, because it does not
perform one. The Shield was measured against a probe that silently encoded
the absence of the very check that would have caught this.

That is M-01's own lesson, from its `CONTRIBUTING.md`:

> measuring a component does not validate the composition; a probe written
> before a guard exists silently encodes that guard's absence as an
> assumption.

The finding is therefore evidence that M-03 was worth building, not
evidence that M-01 was careless. Composition is where this class of defect
is visible, and M-03 is the first composition.

## What Task 4 does instead

`guarded_ur5e` returns a `GuardedUR5e`: a thin `lerobot.robots.Robot`
subclass wrapping the Shield, which supplies the two missing members by
delegation and **adds no safety logic of any kind**.

- `configure` and `is_calibrated` delegate to the wrapped follower.
- Every other member delegates to the Shield, unmodified.
- The adapter must not clamp, validate, filter or reorder anything. If it
  did, claim (A) would be a claim about the adapter rather than about the
  kernel.

The bit-identity assertion is unaffected: it still compares what reaches
`servoJ` against what the kernel emitted. The adapter sits above the
Shield, not between the Shield and the driver.

## What this means for M-01

M-01 is published and this repository does not edit it. The finding is
recorded here and belongs in M-01's `LIMITATIONS.md` as a defect found by
M-03. It is not a safety defect: the Shield's guards all still run. It is
an integration defect, and it means the sentence "the Shield wraps a
LeRobot robot" was never true of the real base class.
