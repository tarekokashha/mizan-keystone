# The staleness guard was blind through the composition

Found by Task 4, which is the only place it was visible, and closed in the
same pass. Everything below was measured, not reasoned about.

## The defect

`sentinel`'s guard 2 asks exactly one question: is the driver's own
observation stamp advancing. It deliberately never compares that stamp
against the kernel's clock, because those are two clocks owned by two
processes with no shared epoch. M-01 learned that the hard way: the two
default Windows clocks were found 5.558460 s apart, which latched a
permanent STOP against a healthy arm.

`UR5eFollower.get_observation` answered that question with
`time.perf_counter()`, the host clock. The host clock advances whether or
not the RTDE stream does. So a controller whose stream had stalled still
produced a stamp marching forward, and `stale` could not fire.

Measured, with the fake's joint data held still:

```
joint data identical across reads: True
stamp strictly increasing        : True
deltas (s): 0.000051100 0.000016900 0.000023300 0.000015000
```

And through the unpatched composition, `stale_replay` fired
`qdd_max, qddd_max, tcp_box, tcp_speed` and never `stale`, despite `stale`
being the guard it declares.

## Why neither programme could have caught it alone

M-01 could not: its fake driver could freeze its stamp on request, because
the fake owned the stamp. A real driver reading a host clock cannot. The
probe was able to express a stall that the shipped driver could never
produce, so the guard passed its test and was inert in the field.

M-03 could not have caught it before Task 4 either. Tasks 1 to 3 test a
driver with no kernel attached, and a stamp nothing reads is just a float.

The defect lives in neither component. It lives at the seam, which is what
M-03 exists to test, and it is the second instance in this programme of
M-01's own lesson: measuring a component does not validate the composition.

## The fix

The driver stamps from `RTDEReceiveInterface.getTimestamp()`, the
controller's own sample clock, which stands still exactly when the stream
does. Verified present on the real installed interface, not assumed:

```
real interface timestamp-ish members: ['getTimestamp']
```

Changing the epoch is safe, and this was checked in the kernel source
rather than taken on trust: guard 2 compares `t_src` only against
`self._t_src_prev`, its own previous value, and never against `now`.

The fake needed a matching change, because it could not model the condition
either: its `getTimestamp` advanced one control period per call
unconditionally. `_SimClock` gained a `stream_frozen` flag, and
`FakeRTDE.freeze_stream()` stalls the stream so that both the timestamp and
the joint data stand still, which is what a stalled controller actually
does. Freezing only the timestamp would have modelled a stall no real
controller produces.

`tests/test_guarded.py` no longer monkeypatches `time` inside the driver to
simulate the stall. It stalls the stream, which is the real condition.

## Evidence the fix is real

Restoring the host-clock stamp fails two tests, with the diagnostic the
regression test was written to give:

```
FAILED tests/test_guarded.py::test_no_attack_reaches_servoj_unauthorised[stale_replay]
FAILED tests/test_guarded.py::test_the_driver_stamp_freezes_with_the_stream_so_stale_can_fire

AssertionError: the driver stamp still advances while the RTDE stream is
stalled; it is reading a host clock again
assert 208025.6303402 == 208025.63032
```

Suite: 90 passed with the fix in place.

## Scope

This was not a safety hole on its own. The kernel's tracking guard still
saw commanded and actual positions diverge, so a stalled stream was not
silently obeyed forever. It did mean one of the ten declared guards was
decorative in any real deployment, and a guard that cannot fire is worse
than a missing one, because the envelope table claims it is there.

Nothing in the kernel, the Shield or the envelope was weakened to achieve
this. The change is entirely in `keystone/`.
