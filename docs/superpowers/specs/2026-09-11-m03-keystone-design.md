# M-03 KEYSTONE: design

Programme: M-03 KEYSTONE (the UR5e follower, and the safety layer in front of it)
Owner: Tarek
Date: 2026-09-11
Status: approved design, pre-implementation

## 1. Why this exists

Two reasons, and the second is the one that matters.

**The obvious one.** `lerobot_ur/robot_ur5e.py` in the MIZAN kit is a skeleton.
Its own header says the LeRobot base-class wiring "is the first overnight task".
It has never been run, never been tested, and is not a LeRobot plugin: it
declares the right method names and satisfies no abstract API.

**The one that matters.** M-01 SENTINEL shipped with a stated limitation, and it
is the largest thing that programme could not verify about itself:

> The Shield has never been composed with the driver it exists to wrap.
> `lerobot_ur/robot_ur5e.py` lives in a separate repository this programme does
> not touch, so every Shield test runs against a fake implementing the LeRobot
> dict contract. What is verified is that the Shield honours that contract, not
> that the real driver honours it too, or that the composition works end to end.

M-03 is where that closes. A safety kernel that has never been attached to a
driver is a safety kernel with an untested assumption at its most important
seam.

## 2. Claim under test

Two claims, deliberately separated, because one is a property of software and the
other is a property of this machine.

> **(A) Authorisation, exact.** No action reaches the RTDE control interface that
> the SENTINEL kernel did not authorise. For any sequence of policy actions,
> including malformed and adversarial ones, every value passed to `servoJ` is a
> value the kernel emitted, unmodified.

> **(B) Timing, measured.** The RTDE control loop sustains a rate and jitter that
> are **measured on the platform under test and reported**, not asserted in
> advance.

(A) is falsifiable by a single counterexample and is testable exhaustively with
no hardware. (B) is a measurement, and the design refuses to pretend otherwise;
see section 6.

## 3. Scope boundary

M-03 builds the follower and the composition. It does not run adversarial
instructions against anything, and it does not touch hardware. The MIZAN
operating contract's gate still applies:

> Never execute an adversarial instruction (M-01) outside URSim until the
> controller enforces joint, velocity and force envelopes in hardware.

M-01 built the enforcement. M-03 attaches it to a driver and proves the
attachment. Running the adversarial half against URSim is a separate programme
with its own protocol.

## 4. Architecture

The central decision is a subtraction.

```
policy / teleop / LeRobot
          |
          v
    Shield  (M-01 SENTINEL, installed as a dependency)
          |
          v
    UR5eFollower  (this repository, NO safety logic at all)
          |
          v
    RTDEControlInterface  |  FakeRTDE
```

### The driver does no safety, on purpose

The skeleton's `send_action` contains six defects, all of them safety logic: a
velocity clamp wrong by the ratio of two rates, no position limits, an unsound
force check, no finiteness validation, no watchdog, and no cumulative guard.

**None of them are fixed in this repository.** They are *removed*. The driver's
job is to move the arm and report its state. Safety is the Shield's job, and it
is already built, reviewed and measured.

That is the architectural claim: safety does not belong in the vendor driver's
diff. A driver with its own half-correct clamps is worse than one with none,
because it invites the reader to trust it.

This also means the driver is small enough to be obviously correct, which is the
only kind of correct that matters at this layer.

### Modules

| module | responsibility | depends on |
|---|---|---|
| `keystone/config.py` | `UR5eConfig`: connection, rates, servo parameters, provenance | none |
| `keystone/rtde_fake.py` | deterministic in-process fake of the ur_rtde interfaces | numpy |
| `keystone/follower.py` | `UR5eFollower`: a real LeRobot plugin, no safety logic | lerobot, ur_rtde |
| `keystone/guarded.py` | factory composing Shield + follower from one config | mizan-sentinel |
| `keystone/timing.py` | rate and jitter measurement, reported not asserted | numpy |
| `keystone/ursim.py` | URSim lifecycle, skipped cleanly when Docker is absent | none |

### `rtde_fake.py` is what makes this testable on Windows

A deterministic fake of `RTDEControlInterface`, `RTDEReceiveInterface` and
`DashboardClient`, recording every call. It is not a mock in the assert-on-calls
sense; it is a tiny simulator that returns plausible joint states and remembers
exactly what was commanded.

With it, claim (A) is testable exhaustively, offline, on any platform, in
milliseconds. Without it, every test needs Docker and a 500 Hz socket, which is
how a project ends up with a driver nobody tests.

## 5. What the LeRobot contract actually is

The skeleton guesses at it. This repository will not: `lerobot` 0.4.4 is
installed as a dependency and the plugin is written against the real base class,
with a test that fails if the installed version's abstract API changes.

The skeleton's header lists three community plugins to credit and consolidate.
Those are prior art and will be cited in the README, not copied.

## 6. Timing, and why the skeleton's gate is not adopted

`ursim_smoke.py` declares a CI gate of "rate >= 480 Hz, p99.9 jitter < 2500 us
on a laptop; tighten once you have PREEMPT_RT".

That number was written without running it, on a platform that was not specified.
This programme does not adopt a threshold it has not measured. The kit's own
operating contract forbids writing a robot number that was not measured, and a
latency gate is a robot number.

So: `timing.py` measures rate and inter-sample jitter at p50, p99 and p99.9 and
**reports** them. The CI gate is set from an observed distribution on a named
platform, with the platform recorded beside it, after the measurement exists.

There is a real possibility that Windows with Docker Desktop cannot sustain the
skeleton's numbers, because the RTDE stream crosses a NAT and the host has no
real-time scheduling. If so, that is a finding about the platform and it will be
reported as one. It is not a reason to relax a threshold quietly, and it is not a
reason to pretend the measurement did not happen.

## 7. Testing

Test-driven throughout, a failing test before every unit.

- **Claim (A), exhaustively, against `FakeRTDE`.** Every adversarial action from
  SENTINEL's own fifteen-attack catalogue is driven through the composed stack,
  and the assertion is that the value reaching `servoJ` is bit-identical to the
  value the kernel emitted. This is the test M-01 could not write.
- **The plugin contract**, against the installed `lerobot` base class.
- **The fake's own fidelity**: it must reject calls the real interface rejects,
  in the cases the driver depends on.
- **Property tests** over action sequences, reusing the `hypothesis` strategies
  that M-01 established, including the stoppability precondition.
- **URSim integration**, skipped with a clear reason when Docker is unavailable
  rather than failing.

CI runs on `windows-latest` and `ubuntu-latest` for everything that does not need
Docker, and adds a URSim job that is allowed to be the only place a run is slow.

## 8. Deliverables

```
mizan-keystone/
  PROTOCOL.md          pre-registration, committed before the timing run
  README.md
  LIMITATIONS.md
  tasks.ps1
  pyproject.toml       installable, unlike M-01 was
  keystone/            config, rtde_fake, follower, guarded, timing, ursim
  tests/
  results/             measured timing distributions
  docs/superpowers/
```

## 9. Out of scope

No adversarial instruction execution. No real hardware. No Robotiq gripper
driver beyond the interface (the skeleton's placeholder stays a placeholder, and
says so). No camera integration. No PREEMPT_RT tuning. No attempt to make
Windows real-time.

## 10. Risks

- **URSim may not be reachable.** Docker Desktop was not running when this design
  was written. If it cannot be brought up, claims (A) and the whole offline test
  suite still hold; (B) becomes unmeasured and is reported as unmeasured.
- **The measured timing may be poor.** Expected on this platform. It is a
  finding, not a failure, provided it is reported as measured rather than
  asserted.
- **`lerobot`'s plugin contract moves between releases.** The skeleton's header
  says as much. Pinning the version and testing against the installed base class
  is the mitigation, and the pin is recorded.
- **M-01's kernel is now a dependency.** A defect there reaches here. That is the
  correct direction for a safety kernel and the reason it was packaged rather
  than vendored.
