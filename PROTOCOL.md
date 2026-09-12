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

Run executed 2026-09-12 against harness `3f9e5fb`, at the budget declared in
section 5 and with no deviation from sections 1 to 7. Raw data:
`results/timing.json`.

```
run                         rate Hz     p50 us     p99 us   p99.9 us     max us
driver_only r1              21353.5      39.20     112.80     290.11  140069.70
composed r1                  1303.1     776.60    1754.33    3874.60  135282.40
driver_only r2              19654.9      34.80      96.11     284.10  161196.70
composed r2                  1489.9     671.95    1353.30    3457.52  131610.90
driver_only r3              22870.4      36.40     123.70     361.10  127086.00
composed r3                  1398.0     723.50    1489.58    3654.50  141908.30
```

All three repeats are reported, as declared. None was discarded.

### What the numbers say

**The safety layer costs about 0.64 ms to 0.74 ms per cycle at the median.**
Composed p50 is 671.95 to 776.60 us against a driver-only p50 of 34.80 to
39.20 us. That difference is the SENTINEL kernel and Shield doing their
work, measured rather than asserted, and it is the first time that cost has
been quantified at all.

**The composed stack sustains 1303 to 1490 Hz on software cost alone.** So
the kernel is not too slow to sit inside a control loop. At the configured
125 Hz, a period of 8000 us, even the p99.9 of 3457 to 3875 us fits inside
the period with room left.

**But the p99.9 already exceeds a 500 Hz deadline.** A 500 Hz period is 2000
us and the composed p99.9 is 3457 to 3875 us in every repeat. On this
platform, at 500 Hz, roughly one cycle in a thousand would overrun its
deadline on software cost alone, before any network is involved.

**And the extreme tail belongs to the platform, not to the kernel.** Max is
127.09 to 161.20 ms, and it is the same order in the driver-only runs
(127.09 to 161.20 ms) as in the composed runs (131.61 to 141.91 ms). A
bare driver doing almost nothing shows the same 100 ms class outliers, so
they are the operating system descheduling the process, not the safety
layer. That attribution is only available because both configurations were
measured; timing the composed stack alone would have made the kernel look
responsible for them.

### The kit's gate, now that a measurement exists

`ursim_smoke.py` declares "rate >= 480 Hz, p99.9 jitter < 2500 us on a
laptop". Measured on the platform in section 7, the composed stack passes
the rate half comfortably and **fails the p99.9 half in all three repeats**,
at 3457.52, 3654.50 and 3874.60 us against a 2500 us bound.

Per section 6, that is a finding about this platform and it is reported as
one. The gate is not relaxed to accommodate it, and no gate is adopted here.
It is worth being precise about what failed: this is the software cost
against a fake, so the real figure including RTDE can only be worse. A
Windows host with no real-time scheduling is not a platform on which a 500
Hz hard deadline should be claimed, and this measurement is the evidence for
saying so rather than a reason to argue with the bound.

### Still unmeasured

End-to-end RTDE timing, for the reason given in section 2: URSim is
unreachable on this host. Nothing above describes it. The half of claim (B)
that requires a controller remains unmeasured and is reported as unmeasured.

## 10. Deviations after the outcome

(none)

## 11. Addendum: URSim, measured after all

Section 2 recorded end-to-end RTDE timing as unmeasured because the Docker
engine on the development host never answered. That remained true of that
host. It stopped being true of the programme once CI showed a working engine
on a GitHub ubuntu runner, so the measurement was taken there rather than
left undone. Nothing in sections 1 to 7 was changed to accommodate it.

Observed on `ubuntu-latest`, image `universalrobots/ursim_e-series`:

```
URSim ready: RTDE session established after 4 attempts
URSim powered: robot mode Robotmode: RUNNING
  samples              : 2000
  wall elapsed s       : 2.148188
  read rate Hz         : 931.0
  controller dt p50 s  : 0.008000
  read gap p50 us      : 1072.45
  read gap p99 us      : 1120.54
  read gap p99.9 us    : 1196.14
```

**The one figure that means what it appears to mean is `controller dt p50`.**
It is 0.008000 s, exactly 125 Hz, and it is the controller's own sample
period read from its own clock. That is an independent confirmation that the
rate `UR5eConfig.declared()` carries is the rate a real controller serves.

**The read gap figures do not measure RTDE latency and must not be quoted as
if they did.** The sampling loop contains a deliberate 1 ms sleep, for the
reason in section 3's spirit: an earlier attempt read 2000 timestamps in a
few milliseconds, landed entirely inside one controller sample, and
concluded the stream was frozen when it was merely slower than the reader.
A p50 read gap of 1072.45 us is therefore that 1 ms sleep plus overhead. It
describes this test's pacing, not the controller's.

### What is still unmeasured, and why

The **control** direction. `RTDEControlInterface` uploads a control script
and requires the teach pendant in remote control mode, which headless URSim
does not offer:

```
RuntimeError: Failed to start RTDE data synchronization, before timeout
```

So `servoJ` has still never been issued to a controller, simulated or real.
Claim (A) remains established against the deterministic fake only. The test
skips on exactly that message and fails on any other RuntimeError, so the
gap cannot silently widen into an absorbed regression.
