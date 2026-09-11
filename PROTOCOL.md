# PROTOCOL.md  (pre-registration; commit before the timing run)

Programme: M-03 KEYSTONE
Owner: Tarek
Date committed: 2026-09-12
Git hash of the specification this protocol argues from: `804f9d9`

**Sequencing.** This file is committed before any timing measurement has been
taken and before `keystone/timing.py` exists. The claims below are the ones
fixed in the design specification,
`docs/superpowers/specs/2026-09-11-m03-keystone-design.md`, committed
2026-09-11. Nothing here is written to fit a result, because no result exists
yet. Deviations made after this point are recorded in section 8 by appending,
never by editing the sections above them.

## 1. Claims under test

Two claims, deliberately separated, because one is a property of software and
the other is a property of this machine.

> **(A) Authorisation, exact.** No action reaches the RTDE control interface
> that the SENTINEL kernel did not authorise. For any sequence of policy
> actions, including malformed and adversarial ones, every value passed to
> `servoJ` is a value the kernel emitted, unmodified.

Falsifiable by a single counterexample. Tested exhaustively offline against
the deterministic fake, driving all fifteen attacks from SENTINEL's own
catalogue. Bit-identity is asserted with `numpy.array_equal`, not
`allclose`: a tolerance here would be a tolerance on whether the safety
layer was bypassed, which is not a thing that admits tolerance.

> **(B) Timing, measured.** The control loop sustains a rate and jitter that
> are measured on the platform under test and reported, not asserted in
> advance.

(B) is not falsifiable and is not meant to be. It is a measurement. It can be
reported wrongly, and sections 3 to 6 exist to constrain that.

## 2. What is measured, and what is explicitly not

This distinction is the most important thing in this document, because
conflating the two would let a fast number stand in for a number that was
never taken.

**Measured:** the wall-clock cost of one full cycle of the composed software
stack, policy action in to `servoJ` call out, with the RTDE interface replaced
by the deterministic in-process fake. This answers a real and useful question:
whether the SENTINEL kernel and Shield are fast enough to sit inside a control
loop at all, and what they cost per cycle.

**Not measured:** end-to-end RTDE timing against a controller. That requires
URSim or hardware. It crosses a socket, a NAT and a scheduler that this
measurement does not touch. No number produced by this protocol describes it.

**Status of the unmeasured half, recorded now rather than later.** URSim is
unavailable on this host at the time of writing. Docker Desktop's processes
are running, but `docker version`, `docker ps` and `docker info` each return
no output and no error and are killed by a timeout. The engine is not serving
its API. Per design specification section 10, that makes (B) partially
unmeasured, and it is reported as unmeasured. It is not grounds for relaxing
anything, and it is not grounds for presenting the fake-backed number as if it
were the RTDE number.

## 3. The clock, and why this section exists

Pre-registered because choosing the wrong clock would silently produce jitter
figures that are pure quantisation artefact, and they would look plausible.

All timing uses `time.perf_counter()`. Measured on this platform with
`time.get_clock_info`:

```
perf_counter: resolution=0.000000100s  impl=QueryPerformanceCounter()
monotonic   : resolution=0.015625000s  impl=GetTickCount64()
time        : resolution=0.015625000s  impl=GetSystemTimeAsFileTime()
```

`time.monotonic()` on Windows resolves to 15.625 ms. A 500 Hz control period
is 2 ms. The default monotonic clock is therefore about eight times coarser
than the entire interval it would be measuring, and every sub-period jitter
figure taken with it would be a multiple of 15.625 ms or zero. `perf_counter`
resolves to 100 ns and is the only defensible choice here.

M-01 SENTINEL found the same two clocks disagreeing by 5.558460 s on this
machine, which latched a permanent STOP against a healthy arm. Two clocks are
not interchangeable just because both are called monotonic.

## 4. Primary metrics

Reported per measurement run:

- Achieved rate, in Hz, over the whole run.
- Inter-sample interval distribution: p50, p99, p99.9, and max.
- Sample count actually collected.

Reported as a distribution. A single mean is not an acceptable summary of
jitter, because the tail is the part that breaks a control loop.

## 5. Budget, fixed now

- 20000 cycles per configuration, fixed in advance.
- A warmup of 1000 cycles is discarded before collection begins, declared
  here rather than chosen after seeing the data.
- No stopping rule. The budget is exhausted regardless of what the numbers
  look like along the way. There is no "rerun until it looks clean".
- The run is repeated 3 times and all 3 are reported, not the best.

## 6. Decision rule, and the threshold this protocol refuses to set

**No pass/fail threshold is defined here, and `keystone/timing.py` will
contain no threshold.**

The vendor kit's `ursim_smoke.py` declares a CI gate of "rate >= 480 Hz,
p99.9 jitter < 2500 us on a laptop; tighten once you have PREEMPT_RT". That
number was written without running it, on a platform that was not specified.
This programme does not adopt a threshold it has not measured. The kit's
operating contract forbids writing a robot number that was not measured, and
a latency gate is a robot number.

A CI gate may be set only after a distribution exists, only from that
observed distribution, and only with the platform recorded beside it. If the
measured numbers are poor, that is a finding about this platform and it is
reported as one. Windows with Docker Desktop and no real-time scheduling may
well fail to sustain the kit's figures. That would be a fact, not a failure,
and it must not be resolved by quietly loosening a bound.

## 7. Platform under test, measured

Recorded because a timing number without a platform is not a measurement.

```
cpu    : 11th Gen Intel(R) Core(TM) i5-11400H @ 2.70GHz
cores  : 6 physical, 12 logical
ram    : 15.7 GB
os     : Windows 10.0.26200
python : 3.11.15
```

No real-time scheduling. No PREEMPT_RT. This is a general purpose laptop
running an ordinary desktop session, and other processes were running during
any measurement taken on it. That is disclosed rather than controlled for.

## 8. Deviations

Appended, never overwritten. If a deviation is made, it is recorded here with
the date and the reason, and the sections above stay as they were committed.

(none yet)

## 9. Outcome

To be completed after the run, by appending. Empty at commit time, which is
the point.
