"""Tests for keystone.follower: UR5eFollower, a real LeRobot plugin with no
safety logic.

UR5eFollower implements lerobot.robots.Robot's abstract API, measured
directly against the installed lerobot 0.4.4 rather than copied from the
vendor kit's skeleton (see docs/decisions/task-1-2-correction.md, Finding 1):
ten abstract members, not the skeleton's eight. A subclass with only the
skeleton's eight could not be instantiated at all.

The most important test in this file is
test_send_action_has_no_safety_logic_a_huge_action_reaches_servoj_unmodified.
The driver carries no clamp, no limit, no finiteness check, no watchdog and
no cumulative guard on purpose (docs/superpowers/specs/2026-09-11-m03-
keystone-design.md, section 4, "The driver does no safety, on purpose"). That
test states the architecture in code so nobody "fixes" what looks like a
missing safety check. Safety is sentinel.Shield's job; Task 4 composes it in
front of this driver.

A test that states an architecture is only worth what it can detect. That
one named six defects and, measured by re-introducing each into send_action
and running the suite, caught one of them: a single finite action of
magnitude 1e9 on a freshly built follower can only expose a transformation
that is unconditional, stateless and triggered below 1e9. The stimulus is
now a sequence, non-finite as well as absurd, driven with a clock that jumps
and a loaded TCP force, and each of the eight known mutations was applied
one at a time and observed to fail it. The table is in that test's own
docstring.
"""
from __future__ import annotations

import itertools
import time

import numpy as np
import pytest
from lerobot.robots import Robot
from lerobot.utils.errors import DeviceNotConnectedError

import keystone.follower as follower_module
from keystone.config import UR5eConfig
from keystone.follower import UR5eFollower
from keystone.rtde_fake import FakeRTDE


def _make(**cfg_overrides) -> tuple[UR5eFollower, FakeRTDE]:
    cfg = UR5eConfig.declared()
    if cfg_overrides:
        cfg = cfg.replace(**cfg_overrides)
    fake = FakeRTDE(control_hz=cfg.control_hz)
    return UR5eFollower(cfg, rtde=fake), fake


def _connected(**cfg_overrides) -> tuple[UR5eFollower, FakeRTDE]:
    """A follower that has been through connect().

    Every I/O test needs one now: Robot.is_connected's contract is that
    get_observation and send_action raise while is_connected is False, which
    test_satisfies_the_installed_abstract_api asserts. connect() touches the
    dashboard only, so a test that counts control or receive calls after
    this sees nothing extra from it.
    """
    follower, fake = _make(**cfg_overrides)
    follower.connect()
    return follower, fake


def _bits(values) -> bytes:
    """The raw float64 bytes of a joint vector.

    The claim about this driver is that what reaches servoJ is bit-identical
    to what was asked for, so bits are what gets compared. Two reasons over
    ==: it is exact for NaN, which no equality operator is, and it cannot be
    satisfied by a substituted value that merely compares close.
    """
    return np.asarray(values, dtype=np.float64).tobytes()


# ---- the corrected LeRobot contract ---------------------------------------- #

def test_satisfies_the_installed_abstract_api():
    # lerobot.robots.Robot declares ten abstract members (see
    # docs/decisions/task-1-2-correction.md). The vendor skeleton implemented
    # eight and raised TypeError on instantiation. This must not repeat that.
    assert issubclass(UR5eFollower, Robot)
    assert UR5eFollower.__abstractmethods__ == frozenset()
    follower, fake = _make()  # raises TypeError if any abstract member is missing
    assert isinstance(follower, Robot)

    # Implementing the members is the smaller half of the contract. Robot's
    # own docstring for is_connected states behaviour too: when it is False,
    # calling get_observation or send_action "should raise an error". This
    # is the test whose name claims the whole installed API is satisfied, so
    # it asserts that half as well.
    assert follower.is_connected is False
    with pytest.raises(DeviceNotConnectedError):
        follower.get_observation()
    with pytest.raises(DeviceNotConnectedError):
        follower.send_action({"joint_position": [0.0] * 6})
    assert fake.control.calls == []
    assert fake.receive.calls == []


def test_config_class_and_name_are_set():
    # Robot.__init__ reads self.name before __init__ runs (robot_type,
    # calibration_dir) and Robot's docstring requires config_class and name
    # to be set in every subclass.
    assert UR5eFollower.config_class is UR5eConfig
    assert UR5eFollower.name == "ur5e"


# ---- feature contracts ------------------------------------------------------ #

def test_observation_features_match_the_declared_shapes():
    follower, _ = _make()
    assert follower.observation_features == {
        "joint_position": (6,),
        "joint_velocity": (6,),
        "tcp_pose": (6,),
        "tcp_force_torque": (6,),
        "gripper_position": (1,),
        "timestamp_monotonic": float,
    }


def test_action_features_match_the_declared_shapes():
    follower, _ = _make()
    assert follower.action_features == {"joint_position": (6,), "gripper_position": (1,)}


# ---- lifecycle: connect and disconnect drive the dashboard in order --------- #

def test_connect_drives_the_dashboard_in_order_and_flips_is_connected():
    follower, fake = _make()
    assert follower.is_connected is False

    follower.connect()

    assert follower.is_connected is True
    assert [c.method for c in fake.dashboard.calls] == ["connect", "powerOn", "brakeRelease"]


def test_disconnect_stops_the_control_loop_before_dropping_the_dashboard():
    follower, fake = _make()
    follower.connect()

    follower.disconnect()

    assert follower.is_connected is False
    # servoStop and stopScript must happen, and in that order, before the
    # control channel itself disconnects; the dashboard drops last.
    assert [c.method for c in fake.control.calls][-3:] == ["servoStop", "stopScript", "disconnect"]
    assert fake.receive.calls[-1].method == "disconnect"
    assert [c.method for c in fake.dashboard.calls] == ["connect", "powerOn", "brakeRelease", "disconnect"]


def test_is_calibrated_is_always_true():
    # UR arms are factory calibrated. Robot's own docstring for
    # is_calibrated says it "should be always True if not applicable", and
    # calibration does not apply to this driver.
    follower, _ = _make()
    assert follower.is_calibrated is True


def test_calibrate_and_configure_are_callable_no_ops():
    # Robot requires both hooks to exist. Neither has anything to do here:
    # calibration does not apply, and there is no one-time RTDE
    # configuration beyond what connect() already does.
    follower, _ = _make()
    assert follower.calibrate() is None
    assert follower.configure() is None


# ---- the connection contract the base class states -------------------------- #

def test_get_observation_on_a_disconnected_robot_raises_and_reads_nothing():
    """Robot.is_connected's contract, in the direction that matters.

    This is transport state, not safety logic. The check reads is_connected
    and nothing else; it does not look at an action at all. The
    no-safety-logic claim is about what this driver does to the values in an
    action, and refusing to talk to an interface that was never opened does
    nothing to any value.
    """
    follower, fake = _make()

    with pytest.raises(DeviceNotConnectedError):
        follower.get_observation()

    assert fake.receive.calls == []


def test_send_action_on_a_disconnected_robot_raises_and_reaches_no_servoj():
    follower, fake = _make()

    with pytest.raises(DeviceNotConnectedError):
        follower.send_action({"joint_position": [0.1] * 6})

    assert fake.control.calls == []


def test_send_action_after_disconnect_raises_rather_than_servoing_a_dead_interface():
    """The observed defect, named so a regression is unambiguous.

    Measured before the fix: send_action after disconnect() issued servoJ
    with control.disconnect already called, so the driver was commanding
    motion through an interface it had itself torn down.
    """
    follower, fake = _connected()
    follower.send_action({"joint_position": [0.0] * 6})
    follower.disconnect()
    assert "disconnect" in [c.method for c in fake.control.calls]

    mark = len(fake.control.calls)
    with pytest.raises(DeviceNotConnectedError):
        follower.send_action({"joint_position": [0.0] * 6})
    with pytest.raises(DeviceNotConnectedError):
        follower.get_observation()

    assert fake.control.calls_since(mark) == []


# ---- get_observation returns every declared key ------------------------------ #

def test_get_observation_returns_every_declared_key_with_the_declared_shape():
    follower, _ = _connected()

    obs = follower.get_observation()

    assert set(obs) == set(follower.observation_features)
    for key in ("joint_position", "joint_velocity", "tcp_pose", "tcp_force_torque"):
        assert obs[key].shape == (6,)
        assert obs[key].dtype == np.float64
    assert obs["gripper_position"].shape == (1,)
    assert obs["gripper_position"].dtype == np.float64
    assert isinstance(obs["timestamp_monotonic"], float)


def test_get_observation_stamps_from_the_controller_sample_clock_not_a_host_clock(monkeypatch):
    """The stamp is the controller's own sample clock, and it stalls with it.

    This replaces a test that asserted only t1 >= t0, which a host clock, a
    controller clock and the literal 0.0 all satisfy. Its name and its
    comment were entirely about which clock was read, and it could not
    detect the clock changing at all.

    The property that matters is not which host clock is used. It is that
    the stamp is not a host clock. sentinel's staleness guard asks one
    question, whether the driver's observation stamp is advancing, and
    compares that stamp only against its own previous value. A host clock
    advances whether or not the RTDE stream does, so a stalled controller
    still looked fresh and `stale` could never fire through the real driver.
    Found by composing the two programmes and recorded in
    docs/decisions/task-4-staleness-is-blind.md.

    Four assertions, and each refutes a different wrong stamp: the stream is
    asked, the value handed back is that answer verbatim, a live stream
    advances it by one sample, and a stalled stream stops it dead.
    """
    follower, fake = _connected()

    # 1. The stream is read, once, on every observation.
    mark = len(fake.receive.calls)
    follower.get_observation()
    assert [c.method for c in fake.receive.calls_since(mark)].count("getTimestamp") == 1

    # 2. The stamp is that read, verbatim. A value no host clock returns
    #    makes that unambiguous: perf_counter and monotonic are positive and
    #    a hardcoded 0.0 is not this. A marker, not a measurement.
    monkeypatch.setattr(fake.receive, "getTimestamp", lambda: -4242.5)
    stamped = follower.get_observation()["timestamp_monotonic"]
    monkeypatch.undo()
    assert stamped == -4242.5
    assert isinstance(stamped, float)

    # 3. A live stream advances it by exactly one control period, which is
    #    what one RTDE sample is.
    a = follower.get_observation()["timestamp_monotonic"]
    b = follower.get_observation()["timestamp_monotonic"]
    assert b > a
    assert b - a == pytest.approx(1.0 / follower.config.control_hz)

    # 4. A stalled stream freezes it. This is the assertion a host clock
    #    cannot pass, and the reason this test exists.
    fake.freeze_stream()
    c = follower.get_observation()["timestamp_monotonic"]
    d = follower.get_observation()["timestamp_monotonic"]
    assert d == c, (
        "the driver stamp still advances while the RTDE stream is stalled; "
        "it is reading a host clock again")


# ---- send_action: pass-through, exactly, no safety logic --------------------- #

def test_send_action_passes_the_joint_target_to_servoj_unmodified():
    follower, fake = _connected()
    q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

    result = follower.send_action({"joint_position": q})

    servoj_calls = [c for c in fake.control.calls if c.method == "servoJ"]
    assert len(servoj_calls) == 1
    assert servoj_calls[0].args[0] == q
    assert result["joint_position"] == q


def test_send_action_returns_exactly_what_was_sent():
    # "take the action, call servoJ, return what was sent": every key in the
    # action, not only joint_position, comes back unchanged.
    follower, _ = _connected()
    action = {"joint_position": [0.0] * 6, "gripper_position": [0.5]}

    result = follower.send_action(action)

    assert result == action


@pytest.mark.parametrize("overrides", [
    {},
    {"control_hz": 100.0, "servo_lookahead": 0.05, "servo_gain": 500.0},
], ids=["declared", "overridden"])
def test_send_action_passes_every_servoj_argument_from_config(overrides):
    """servoJ's remaining arguments, which nothing used to assert.

    Only argument 0 was checked, so the three that carry configuration were
    free to be anything at all. Each of these was re-introduced and the
    suite stayed green:

        1.0 / self.config.control_hz  ->  self.config.control_hz
        self.config.servo_gain        ->  2000.0
        self.config.servo_lookahead   ->  0.2

    The first is precisely the defect class the spec names in the skeleton,
    a velocity clamp wrong by the ratio of two rates. servoJ blocks for its
    time argument, so a rate where a period was meant is not a small error.
    The other two mean the config's validated values need not arrive at all.

    Two configurations are run, and their values differ from each other and
    from the constants above, so what is asserted is that the config's
    values flow through rather than that some constant happens to match one
    of them. The whole argument tuple is compared, which pins the order too.
    """
    follower, fake = _connected(**overrides)
    cfg = follower.config
    q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

    follower.send_action({"joint_position": q})

    servoj_calls = [c for c in fake.control.calls if c.method == "servoJ"]
    assert len(servoj_calls) == 1
    assert servoj_calls[0].args == (
        q,
        0.0,                       # speed, which servoJ does not use
        0.0,                       # acceleration, which servoJ does not use
        1.0 / cfg.control_hz,      # time: one control period, not the rate
        cfg.servo_lookahead,
        cfg.servo_gain,
    )


def test_send_action_has_no_safety_logic_a_huge_action_reaches_servoj_unmodified(monkeypatch):
    """The test that states the architecture. Do not "fix" this.

    A real joint value of 1e9 would destroy the arm. That is exactly the
    point: this driver does not know that, and it is not supposed to. The
    vendor skeleton's send_action had six defects, all of them safety logic
    guessed at without measurement (a velocity clamp wrong by the ratio of
    two rates, no position limits, an unsound force check, no finiteness
    validation, no watchdog, no cumulative guard). None of it is fixed in
    this repository. It is removed. Safety is sentinel.Shield's job (M-01
    SENTINEL), already built, reviewed and measured; Task 4 composes it in
    front of this driver so nothing reaches servoJ the Shield did not
    authorise. A driver carrying its own half-correct clamp is worse than
    one carrying none, because it invites the reader to trust it. If you
    are reading this because you are about to add a clamp, limit, watchdog
    or finiteness check to keystone.follower.UR5eFollower.send_action:
    stop. Put it in the Shield or the Envelope instead.

    What the stimulus has to be, and why. This test used to fire a single
    finite action of magnitude 1e9 at a freshly built follower, which can
    only expose a transformation that is unconditional, stateless and
    triggered below 1e9. Five of the six defects it names survived being put
    back, measured by mutation. A rate limiter, a watchdog and a cumulative
    budget each need a second call; a force check needs a force, and the
    fake returns zeros; a finiteness check needs a non-finite value. So:

        mutation re-introduced into send_action    detected by
        ----------------------------------------  -------------------------
        np.clip(q, -6.28, 6.28)                   bits, on every action
        non-finite guard returning early          the NaN and inf actions
        rate limit against the previous command   the sign-flipping pair
        force check on getActualTCPForce()        the loaded wrench
        watchdog, servoStop past a threshold      the jumping clock
        cumulative path budget across commands    seven successive commands
        non-finite q replaced by getActualQ()     bits, and no read happens
        drop servoJ when max abs(q) > 1e10        the 1e12 action

    The magnitudes here are deliberately not physical. They sit past any
    threshold a guess would pick, and none of them is a claim about a UR5e.

    Three assertions carry that. Every commanded vector reaches servoJ
    bit-identical, NaN included. The control interface sees exactly
    initPeriod, servoJ, waitPeriod per action and nothing else, so a tripped
    watchdog shows up as a servoStop and a refusal as a missing servoJ. And
    send_action reads nothing back at all, which is what a force check, a
    stream-clock watchdog and the fabricated-pose substitution each have to
    do before they can act.
    """
    follower, fake = _connected()

    # A wrench the driver could refuse if it looked. Patched onto this
    # FakeRTDE instance deliberately rather than into keystone.rtde_fake:
    # that module models no contact physics and returns zeros by design,
    # and it is not this test's file to change.
    forced: list[str] = []

    def _loaded_tcp_force() -> list[float]:
        forced.append("getActualTCPForce")
        return [1.0e6] * 6

    monkeypatch.setattr(fake.receive, "getActualTCPForce", _loaded_tcp_force)

    # A host clock that jumps ten seconds every time anyone asks it
    # anything. Any watchdog reading one sees a gap past any threshold it
    # could have picked, on every call, deterministically and with no sleep.
    seconds = itertools.count(1.0e4, 10.0)
    nanos = itertools.count(10 ** 13, 10 ** 10)
    for attr in ("perf_counter", "monotonic", "time"):
        monkeypatch.setattr(time, attr, lambda: next(seconds))
    for attr in ("perf_counter_ns", "monotonic_ns", "time_ns"):
        monkeypatch.setattr(time, attr, lambda: next(nanos))

    huge = [1e9] * 6
    absurd = [1e12] * 6
    swing_low = [-1e6] * 6
    swing_high = [1e6] * 6                     # a 2e6 step from the one before
    not_a_number = [float("nan")] * 6
    infinite = [float("inf"), float("-inf"), 0.0, 0.0, 0.0, 0.0]
    sequence = [huge, absurd, swing_low, swing_high, not_a_number, infinite, huge]

    mark_control = len(fake.control.calls)
    mark_receive = len(fake.receive.calls)
    results = [follower.send_action({"joint_position": q}) for q in sequence]
    monkeypatch.undo()

    servoj_calls = [c for c in fake.control.calls_since(mark_control) if c.method == "servoJ"]
    assert len(servoj_calls) == len(sequence), (
        f"{len(sequence)} actions were commanded but servoJ saw "
        f"{len(servoj_calls)}; send_action dropped one")
    for q, call, result in zip(sequence, servoj_calls, results):
        assert _bits(call.args[0]) == _bits(q), (
            f"servoJ received {np.asarray(call.args[0], dtype=float)} for a commanded "
            f"{np.asarray(q, dtype=float)}; something in send_action touched it")
        assert _bits(result["joint_position"]) == _bits(q)

    # Exactly the servo triple, once per action, and nothing else.
    assert [c.method for c in fake.control.calls_since(mark_control)] == (
        ["initPeriod", "servoJ", "waitPeriod"] * len(sequence))

    # send_action reads nothing back from the robot, so there is nothing for
    # a force check, a stream-clock watchdog or a pose substitution to read.
    assert fake.receive.calls_since(mark_receive) == []
    assert forced == []


def test_send_action_uses_initperiod_and_waitperiod_around_servoj():
    # servoJ's own contract pairs it with initPeriod/waitPeriod to hold the
    # control loop period; this is correct low-level RTDE API usage, not a
    # safety behaviour, and keystone.rtde_fake implements both for exactly
    # this reason.
    follower, fake = _connected()
    follower.send_action({"joint_position": [0.0] * 6})
    methods = [c.method for c in fake.control.calls]
    assert methods == ["initPeriod", "servoJ", "waitPeriod"]


# ---- rtde=None builds the real ur_rtde interfaces, in connect() -------------- #

def test_rtde_none_constructs_the_real_interfaces_from_config_at_connect(monkeypatch):
    """UR5eFollower(config) with no injected rtde must build the real
    ur_rtde interfaces from config.ip and config.control_hz, not silently
    fall back to a fake, and must build them in connect() rather than in
    __init__.

    The second half is the correction. Construction is not inert: the
    installed ur_rtde 1.6.5 declares

        __init__(self, hostname, frequency=-1.0,
                 flags=<Flags.FLAG_UPLOAD_SCRIPT: 1>, ...)

    measured from its own pybind11 signature, so building the object reaches
    the address and uploads a control script. Doing that in __init__ meant
    UR5eFollower(config) opened the sockets before any caller said connect.

    Verified by monkeypatching the three real classes with recorders, not by
    opening an actual socket: this repository's offline suite runs with no
    hardware and no Docker, and 192.0.2.10 (UR5eConfig.declared()'s ip) is a
    reserved documentation address that nothing listens on.
    """
    calls: dict[str, object] = {}

    class _RecordingControl:
        def __init__(self, hostname, frequency):
            calls["control"] = (hostname, frequency)

    class _RecordingReceive:
        def __init__(self, hostname, frequency):
            calls["receive"] = (hostname, frequency)

    class _RecordingDashboard:
        def __init__(self, hostname):
            calls["dashboard"] = (hostname,)

        def connect(self):
            calls.setdefault("dashboard_calls", []).append("connect")

        def powerOn(self):
            calls.setdefault("dashboard_calls", []).append("powerOn")

        def brakeRelease(self):
            calls.setdefault("dashboard_calls", []).append("brakeRelease")

    monkeypatch.setattr(follower_module.rtde_control, "RTDEControlInterface", _RecordingControl)
    monkeypatch.setattr(follower_module.rtde_receive, "RTDEReceiveInterface", _RecordingReceive)
    monkeypatch.setattr(follower_module.dashboard_client, "DashboardClient", _RecordingDashboard)

    cfg = UR5eConfig.declared()
    driver = UR5eFollower(cfg)  # rtde=None

    assert calls == {}, f"constructing the follower already opened interfaces: {calls}"
    assert driver.rtde is None

    driver.connect()

    assert calls["control"] == (cfg.ip, cfg.control_hz)
    assert calls["receive"] == (cfg.ip, cfg.control_hz)
    assert calls["dashboard"] == (cfg.ip,)
    # Built before the dashboard is driven, which is where they were built
    # before: only the moment changed, not the order of what happens once
    # connect() runs.
    assert calls["dashboard_calls"] == ["connect", "powerOn", "brakeRelease"]
    assert driver.rtde.control is not None
    assert driver.rtde.receive is not None
    assert driver.rtde.dashboard is not None


def test_disconnect_without_connect_opens_nothing():
    # disconnect() on a follower that never connected has nothing to tear
    # down, and must not build the real interfaces just to close them.
    driver = UR5eFollower(UR5eConfig.declared())  # rtde=None

    driver.disconnect()

    assert driver.rtde is None
    assert driver.is_connected is False
