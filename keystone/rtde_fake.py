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
arguments, in .calls. tests/test_rtde_fake.py's fidelity check compares this
module's method surface against the installed ur_rtde 1.6.5, because a fake
that drifts from the real interface is worse than no fake: the rest of the
suite would then pass against a contract that does not exist.
"""
from __future__ import annotations

import datetime
from dataclasses import dataclass, field
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
    called with. args and kwargs are frozen copies (see _freeze below), so
    mutating a list or array after the call cannot rewrite history.
    """

    method: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]


def _freeze(value: Any) -> Any:
    """A defensive copy of a value about to go into a Call record.

    Without this, `control.servoJ(q, ...)` followed by the caller mutating
    `q` in place would silently rewrite what the fake claims was sent, the
    same class of defeat keystone.config and sentinel.envelope guard against
    for their own mutable fields.
    """
    if isinstance(value, np.ndarray):
        return np.array(value, copy=True)
    if isinstance(value, list):
        return list(value)
    return value


class _CallRecorder:
    """Call-recording behaviour shared by the three fakes."""

    def __init__(self) -> None:
        self.calls: list[Call] = []

    def _record(self, method: str, *args: Any, **kwargs: Any) -> None:
        self.calls.append(Call(method, tuple(_freeze(a) for a in args),
                                {k: _freeze(v) for k, v in kwargs.items()}))

    def calls_since(self, mark: int) -> list[Call]:
        """Calls recorded after `mark`, an index into .calls.

        Typical use: `mark = len(fake.control.calls)` taken before the
        action under test, then `fake.control.calls_since(mark)` after it.
        """
        return self.calls[mark:]


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
        self._record("servoJ", q, speed, acceleration, time, lookahead_time, gain)
        self._clock.q_target = np.array(q, dtype=float).reshape(N_JOINTS)
        return True

    def servoStop(self, a: float = 10.0) -> bool:
        self._record("servoStop", a)
        # Real servoStop decelerates the robot to a halt; modelled here as
        # the target collapsing to the current position, so getActualQ
        # stops advancing.
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
        servoJ has not been called yet."""
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
        self._record("getTimestamp")
        t = self._clock.t
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
