"""Deterministic in-process fakes of the three ur_rtde interfaces.

FakeControl, FakeReceive and FakeDashboard stand in for
rtde_control.RTDEControlInterface, rtde_receive.RTDEReceiveInterface and
dashboard_client.DashboardClient. They are not assert-on-calls mocks: they
are a tiny simulator. FakeControl.servoJ records what it was told and sets a
shared target; FakeReceive.getActualQ steps a shared joint position toward
that target on every call, so a control loop that reads position and
commands a new target actually closes, offline, on any platform, with no
socket and no Docker.

FakeRTDE bundles the three behind one shared clock, which is what makes the
closed loop possible: a servoJ call through .control is visible as motion
through .receive because both hold a reference to the same _SimClock.

Every call to every method is recorded on that method's own fake, with its
arguments, in .calls. That history is immutable in both directions: neither
the caller who made the call nor the reader who inspects it afterwards can
rewrite what it says. tests/test_rtde_fake.py's fidelity check compares this
module's method surface against the installed ur_rtde 1.6.5, because a fake
that drifts from the real interface is worse than no fake: the rest of the
suite would then pass against a contract that does not exist.
"""
from __future__ import annotations

import datetime
from collections.abc import Mapping
from dataclasses import dataclass, field
from types import MappingProxyType
from typing import Any, NamedTuple

import numpy as np

N_JOINTS = 6

# How far the simulated joint position moves toward the last commanded
# servoJ target on each getActualQ() sample. This is a property of this
# fake's simulation, not a measurement or a claim about the real UR5e, so it
# is not subject to the config provenance rule (that rule governs
# keystone.config.UR5eConfig, which describes the robot; this constant
# describes only how fast the fake pretends to move).
_STEP_RAD_PER_SAMPLE = 0.05


class Call(NamedTuple):
    """One recorded invocation: the method name and the arguments it was
    called with.

    args and kwargs are frozen copies (see _freeze below), and the freeze
    runs in both directions. The caller cannot rewrite history by mutating
    the list or array it passed in, because the record holds a copy. The
    reader cannot rewrite history by mutating the copy it is handed back
    through .calls, because that copy refuses in-place mutation. The second
    direction is the one that matters for claim (A): a history its reader
    can edit proves nothing about what was sent.
    """

    method: str
    args: tuple[Any, ...]
    kwargs: Mapping[str, Any]


class _FrozenList(list):
    """A list that refuses every in-place mutation.

    A tuple would be the obvious immutable container, but the recorded
    arguments get compared against plain lists by the readers of .calls,
    and a tuple never compares equal to a list. Subclassing list keeps that
    value equality (list.__eq__ compares contents, not types) while closing
    the in-place mutation routes. A determined caller can still reach the
    storage through the unbound list methods; this guards the accident and
    the honest mistake, not sabotage.
    """

    def _refuse(self, *args: Any, **kwargs: Any) -> Any:
        raise TypeError(
            "a recorded Call is immutable: .calls is the evidence base for "
            "claim (A), not scratch space")

    __setitem__ = _refuse
    __delitem__ = _refuse
    __iadd__ = _refuse
    __imul__ = _refuse
    append = _refuse
    clear = _refuse
    extend = _refuse
    insert = _refuse
    pop = _refuse
    remove = _refuse
    reverse = _refuse
    sort = _refuse


def _freeze(value: Any) -> Any:
    """A frozen copy of a value about to go into a Call record.

    Two different defeats are closed here, and both are needed.

    Forward: `control.servoJ(q, ...)` followed by the caller mutating `q`
    in place would silently rewrite what the fake claims was sent, the same
    class of defeat keystone.config and sentinel.envelope guard against for
    their own mutable fields. The copy closes that.

    Backward: a reader of .calls mutating the copy the recorder kept would
    rewrite the record just as effectively, because Call.args is a tuple
    but the objects inside it held the only copy. A read-only array and a
    mutation-refusing list close that.

    The ndarray branch is the one that carries the weight: sentinel's
    kernel emits float64 arrays, so that is the type the recorded history
    behind claim (A) is actually made of. Pinned by
    test_servoj_recording_is_immune_to_the_caller_mutating_its_ndarray_afterwards.
    """
    if isinstance(value, np.ndarray):
        frozen = np.array(value, copy=True)
        frozen.flags.writeable = False
        return frozen
    if isinstance(value, list):
        return _FrozenList(_freeze(v) for v in value)
    return value


class _CallRecorder:
    """Call-recording behaviour shared by the three fakes."""

    def __init__(self) -> None:
        self._calls: list[Call] = []

    @property
    def calls(self) -> tuple[Call, ...]:
        """The recorded history, oldest first.

        A tuple rather than the internal list, so a reader cannot append a
        call that never happened or drop one that did. The Call records
        inside it are frozen too (see _freeze), so the history is immutable
        all the way down.
        """
        return tuple(self._calls)

    def _record(self, method: str, *args: Any, **kwargs: Any) -> None:
        self._calls.append(Call(
            method,
            tuple(_freeze(a) for a in args),
            MappingProxyType({k: _freeze(v) for k, v in kwargs.items()}),
        ))

    def calls_since(self, mark: int) -> list[Call]:
        """Calls recorded after `mark`, an index into .calls.

        Typical use: `mark = len(fake.control.calls)` taken before the
        action under test, then `fake.control.calls_since(mark)` after it.
        """
        return list(self._calls[mark:])


@dataclass
class _SimClock:
    """Simulated joint state and elapsed-time clock, shared between
    FakeControl and FakeReceive so a servoJ call on one is visible as motion
    on the other.
    """

    control_hz: float
    q: np.ndarray
    q_target: np.ndarray | None = None
    qd: np.ndarray = field(default_factory=lambda: np.zeros(N_JOINTS))
    t: float = 0.0
    # When True the RTDE stream is stalled: the controller keeps
    # serving the same sample, so neither the timestamp nor the joint
    # data advances. Without this the fake cannot model a stalled
    # stream at all, and the kernel's staleness guard has nothing to
    # fire on. See docs/decisions/task-4-staleness-is-blind.md.
    stream_frozen: bool = False


class FakeControl(_CallRecorder):
    """Deterministic fake of rtde_control.RTDEControlInterface.

    Implements exactly the 7 methods the driver needs: servoJ, servoStop,
    stopScript, initPeriod, waitPeriod, disconnect, isConnected. Signatures
    are copied from the installed ur_rtde 1.6.5's own docstrings (measured
    directly, since inspect.signature fails on its pybind11 methods); see
    tests/test_rtde_fake.py's fidelity check.
    """

    def __init__(self, clock: _SimClock | None = None, control_hz: float = 125.0) -> None:
        super().__init__()
        self._clock = clock if clock is not None else _SimClock(
            control_hz=control_hz, q=np.zeros(N_JOINTS))
        self._connected = True

    def servoJ(self, q, speed: float, acceleration: float, time: float,
               lookahead_time: float, gain: float) -> bool:
        # Recorded BEFORE the target is set, deliberately. .calls answers
        # "what did the driver put on the wire", not "what did the
        # simulation manage to do with it", so a command this fake cannot
        # simulate still belongs in the history. If a rejected call went
        # unrecorded, a driver could put a command past the evidence base
        # simply by malforming it, and claim (A) would be read from a log
        # with holes in it. An evidence log may over-report; it must never
        # under-report. Pinned by
        # test_servoj_records_the_attempt_even_when_it_cannot_be_simulated.
        self._record("servoJ", q, speed, acceleration, time, lookahead_time, gain)
        # reshape raises ValueError on a q that is not N_JOINTS long. That
        # is simulation bookkeeping, not validation: the fake has nowhere
        # to put a 3-element or 7-element target. It is emphatically NOT a
        # safety check, and no safety check belongs in this class.
        # Non-finite values pass straight through on purpose: this fake is
        # a dumb transport, and the architecture depends on the driver's
        # lack of safety logic staying observable through it. Pinned by
        # test_servoj_accepts_non_finite_targets_because_the_fake_is_a_dumb_transport.
        self._clock.q_target = np.array(q, dtype=float).reshape(N_JOINTS)
        return True

    def servoStop(self, a: float = 10.0) -> bool:
        self._record("servoStop", a)
        # Real servoStop decelerates the robot to a halt; modelled here as
        # the target collapsing to the current position, so getActualQ
        # stops advancing. Collapsing the target IS the behaviour, not an
        # incidental detail: pinned by
        # test_servostop_actually_stops_the_simulated_motion.
        #
        # A non-finite simulated position is absorbing, and servoStop does
        # not clear it: halting AT the current position cannot help when
        # the current position is itself NaN. That is deliberate. To
        # sanitise here would hide from the kernel exactly the non-finite
        # condition its guards exist to catch. Pinned by
        # test_a_nan_target_poisons_the_state_and_servostop_does_not_clear_it.
        # Whether a real UR5e controller would reject a NaN target at the
        # wire is NOT measurable offline, because the real interfaces
        # connect to hardware in their constructors, so their C++
        # validators cannot be reached from this suite. This fake
        # therefore makes no claim either way.
        self._clock.q_target = np.array(self._clock.q, dtype=float)
        return True

    def stopScript(self) -> None:
        self._record("stopScript")

    def initPeriod(self) -> datetime.timedelta:
        self._record("initPeriod")
        return datetime.timedelta(seconds=self._clock.t)

    def waitPeriod(self, t_start: datetime.timedelta) -> None:
        # No real sleep: determinism and offline speed matter more here
        # than wall-clock pacing. The follower's timing is measured
        # separately in keystone.timing against this same fake.
        self._record("waitPeriod", t_start)

    def disconnect(self) -> None:
        self._record("disconnect")
        self._connected = False

    def isConnected(self) -> bool:
        self._record("isConnected")
        return self._connected

    @property
    def last_servoj_q(self) -> np.ndarray | None:
        """The joint target most recently passed to servoJ, or None if
        servoJ has not been called yet.

        A copy, not the internal target. tests/test_guarded.py asserts
        claim (A) through this property for all fifteen attacks, so a
        caller able to reach _clock.q_target through the value handed back
        could rewrite the very thing that claim is read from. Pinned by
        test_last_servoj_q_is_a_copy_not_the_internal_target.
        """
        return None if self._clock.q_target is None else np.array(self._clock.q_target)


class FakeReceive(_CallRecorder):
    """Deterministic fake of rtde_receive.RTDEReceiveInterface.

    Not an assert-on-calls mock: getActualQ steps the shared simulated
    joint position toward whatever FakeControl.servoJ last commanded (or
    holds position if nothing has been commanded yet), so a control loop
    reading position and issuing a new target actually closes. getTimestamp
    advances by 1/control_hz on every call, modelling one RTDE sample per
    call.
    """

    def __init__(self, clock: _SimClock | None = None, control_hz: float = 125.0) -> None:
        super().__init__()
        self._clock = clock if clock is not None else _SimClock(
            control_hz=control_hz, q=np.zeros(N_JOINTS))
        self._connected = True

    def getActualQ(self) -> list[float]:
        self._record("getActualQ")
        c = self._clock
        if c.stream_frozen:
            # A stalled stream serves the same sample again. Stepping
            # the simulation here would make frozen data look fresh.
            return [float(x) for x in c.q]
        target = c.q if c.q_target is None else c.q_target
        delta = target - c.q
        step = np.clip(delta, -_STEP_RAD_PER_SAMPLE, _STEP_RAD_PER_SAMPLE)
        q_new = c.q + step
        dt = 1.0 / c.control_hz
        c.qd = (q_new - c.q) / dt
        c.q = q_new
        return [float(x) for x in c.q]

    def getActualQd(self) -> list[float]:
        self._record("getActualQd")
        return [float(x) for x in self._clock.qd]

    def getActualTCPPose(self) -> list[float]:
        self._record("getActualTCPPose")
        # No forward kinematics is modelled. A fixed placeholder, not a
        # measurement or an estimate of any real TCP pose: this repository
        # never writes a real-robot number, and a fabricated-but-plausible
        # pose is exactly the kind of number that rule exists to forbid.
        return [0.0] * N_JOINTS

    def getActualTCPForce(self) -> list[float]:
        self._record("getActualTCPForce")
        # No contact physics is modelled. Always zero, a placeholder.
        return [0.0] * N_JOINTS

    def getTimestamp(self) -> float:
        """The controller's own sample clock, which is what the real
        RTDEReceiveInterface.getTimestamp returns. Advances by one
        control period per call, modelling one RTDE sample per call,
        unless the stream is stalled, in which case the controller is
        serving a repeated sample and the timestamp stands still.
        """
        self._record("getTimestamp")
        t = self._clock.t
        if not self._clock.stream_frozen:
            self._clock.t += 1.0 / self._clock.control_hz
        return t

    def disconnect(self) -> None:
        self._record("disconnect")
        self._connected = False


class FakeDashboard(_CallRecorder):
    """Deterministic fake of dashboard_client.DashboardClient.

    Implements exactly the 5 methods the driver needs: connect, powerOn,
    brakeRelease, isInRemoteControl, disconnect.
    """

    def __init__(self) -> None:
        super().__init__()
        self._connected = False
        self._remote_control = True

    def connect(self, timeout_ms: int = 2000) -> None:
        self._record("connect", timeout_ms)
        self._connected = True

    def powerOn(self) -> None:
        self._record("powerOn")

    def brakeRelease(self) -> None:
        self._record("brakeRelease")

    def isInRemoteControl(self) -> bool:
        self._record("isInRemoteControl")
        return self._remote_control

    def disconnect(self) -> None:
        self._record("disconnect")
        self._connected = False


class FakeRTDE:
    """The three fakes, sharing one simulated joint clock.

    This is what makes a control loop testable offline: pass a FakeRTDE
    instance wherever the driver expects to reach a real robot, drive
    .control and read .receive, and the state advances the same way it
    would if a real UR5e were closing the loop underneath.
    """

    def __init__(self, control_hz: float = 125.0, q0: Any = None) -> None:
        q0_arr = np.zeros(N_JOINTS) if q0 is None else np.array(q0, dtype=float).reshape(N_JOINTS)
        clock = _SimClock(control_hz=control_hz, q=q0_arr)
        self.control = FakeControl(clock)
        self.receive = FakeReceive(clock)
        self.dashboard = FakeDashboard()
        self._clock = clock

    def freeze_stream(self) -> None:
        """Stall the RTDE stream: the controller goes on serving the
        same sample, so getTimestamp and getActualQ both stand still.
        This is the condition the kernel's staleness guard exists to
        catch, and it cannot be modelled by freezing the host clock,
        because the host clock is not what a stalled controller stops.
        """
        self._clock.stream_frozen = True

    def thaw_stream(self) -> None:
        """Resume the RTDE stream after freeze_stream."""
        self._clock.stream_frozen = False
