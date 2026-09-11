"""Tests for keystone.guarded: the composition, and the test M-01 could not write.

M-01 SENTINEL shipped with this as its largest stated limitation:

    The Shield has never been composed with the driver it exists to wrap.
    lerobot_ur/robot_ur5e.py lives in a separate repository this programme
    does not touch, so every Shield test runs against a fake implementing
    the LeRobot dict contract. What is verified is that the Shield honours
    that contract, not that the real driver honours it too, or that the
    composition works end to end.

This file is where that closes. The central test drives every attack in
sentinel.attacks.REGISTRY through Shield + UR5eFollower + FakeRTDE and
asserts claim (A) from the design document: every value passed to servoJ is
a value the kernel emitted, unmodified, compared with np.array_equal rather
than np.allclose because "unmodified" means bit-identical and nothing less.
When the kernel emits no command at all (Action.q is None), nothing at all
must reach servoJ.

The adapter under test adds no safety logic. It supplies the two abstract
members sentinel.shield.Shield lacks (configure and is_calibrated, measured
in docs/decisions/task-4-shield-is-not-a-lerobot-plugin.md) and delegates
everything else to the Shield untouched. test_the_adapter_is_a_pass_through
proves that by identity, not by resemblance: if the adapter clamped,
validated, filtered or reordered anything, claim (A) would become a claim
about the adapter rather than about the kernel.

Harness fidelity. Two clocks are driven from one simulated clock so the
episode reproduces sentinel.redteam.run_episode's own conditions offline and
deterministically:

  - the kernel's clock, injected through guarded_ur5e(clock=...), the same
    injection point SafetyKernel already provides. This is what lets
    watchdog_starve's declared skip_seconds actually exceed max_dt_s.
  - the driver's observation stamp, which is not injected at all. It comes
    from the controller's own sample clock, so stalling the RTDE stream with
    FakeRTDE.freeze_stream() is what lets stale_replay's declared
    freeze_state actually freeze an observation.

The second of those is a finding in its own right, recorded in
docs/decisions/task-4-staleness-is-blind.md and regression-tested in
test_the_driver_stamp_freezes_with_the_stream_so_stale_can_fire below.
"""
from __future__ import annotations

import numpy as np
import pytest
from lerobot.robots import Robot
from sentinel.attacks import DT, EPISODE_Q0, REGISTRY
from sentinel.envelope import Envelope
from sentinel.kernel import SafetyKernel
from sentinel.shield import Shield
from sentinel.types import RobotState, Status

from keystone.config import UR5eConfig
from keystone.follower import UR5eFollower
from keystone.guarded import GuardedUR5e, guarded_ur5e
from keystone.rtde_fake import FakeRTDE

# Long enough for every guard this composition can reach to fire, short
# enough that fifteen attacks run quickly. sentinel.redteam uses 2000 steps
# x 200 episodes for its statistical claim; claim (A) is falsifiable by a
# single counterexample and needs no statistics, only coverage.
STEPS = 400

# slow_drift is the one attack whose declared guard cannot fire in 400
# steps, and the reason is arithmetic rather than a defect. It declares
# envelope_override={"path_budget_m": 8.0}, and the flange path this
# composition accumulates was measured at 2.854423 m after 400 steps and
# 7.215064 m after 2000, where path_budget does fire. sentinel.redteam runs
# every attack for 2000 steps; here only this one needs them.
STEPS_OVERRIDE = {"slow_drift": 2000}


class _Clock:
    """The simulated clock the kernel is given.

    Mirrors sentinel.redteam._Clock. It drives the kernel only. It used to
    carry a second field for the driver's observation stamp as well, back
    when the driver read a host clock; the driver now stamps from the
    controller's sample clock, which is FakeRTDE's to advance and not this
    object's. See docs/decisions/task-4-staleness-is-blind.md.
    """

    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t

    def tick(self, dt: float = DT) -> None:
        self.t += dt



def _state_from(obs: dict) -> RobotState:
    """The state an attack function is called with.

    Built from the driver's own observation dict, the same fields and the
    same order sentinel.shield.Shield uses to build the state it hands the
    kernel, so the attack and the kernel see one observation and not two.
    """
    return RobotState(
        q=obs["joint_position"],
        qd=obs["joint_velocity"],
        t_mono=float(obs["timestamp_monotonic"]),
        wrench=obs["tcp_force_torque"],
        gripper=float(np.asarray(obs["gripper_position"]).ravel()[0]),
    )


def _payload(action) -> dict:
    """A sentinel Action rendered as the LeRobot action dict a policy sends."""
    out: dict = {"joint_position": action.q}
    if action.gripper is not None:
        out["gripper_position"] = np.array([action.gripper])
    return out


# ---- the adapter is a real LeRobot plugin, and the Shield is not ------------ #

def test_the_shield_alone_is_not_a_lerobot_robot():
    """The measurement behind docs/decisions/task-4-shield-is-not-a-lerobot-plugin.md.

    M-01's Shield implements eight of Robot's ten abstract members and does
    not inherit from Robot. The two it lacks are exactly the two the vendor
    skeleton lacked. This is asserted here rather than only written down, so
    the day sentinel grows them the decision record stops being true and
    this test says so.
    """
    assert not issubclass(Shield, Robot)
    assert sorted(m for m in Robot.__abstractmethods__ if not hasattr(Shield, m)) == [
        "configure", "is_calibrated"]


def test_guarded_ur5e_is_a_real_lerobot_plugin():
    robot = guarded_ur5e(UR5eConfig.declared(), rtde=FakeRTDE())
    assert isinstance(robot, GuardedUR5e)
    assert issubclass(GuardedUR5e, Robot)
    assert GuardedUR5e.__abstractmethods__ == frozenset()
    assert isinstance(robot, Robot)


def test_guarded_ur5e_composes_a_shield_over_the_real_follower():
    fake = FakeRTDE()
    robot = guarded_ur5e(UR5eConfig.declared(), rtde=fake)
    assert isinstance(robot.shield, Shield)
    assert isinstance(robot.shield.robot, UR5eFollower)
    assert robot.shield.robot.rtde is fake
    assert isinstance(robot.shield.kernel, SafetyKernel)


def test_guarded_ur5e_defaults_to_the_declared_envelope_and_accepts_an_override():
    fake = FakeRTDE()
    default = guarded_ur5e(UR5eConfig.declared(), rtde=fake)
    assert default.shield.kernel.env.sha256() == Envelope.ur5e_declared().sha256()

    other = Envelope.ur5e_declared().replace(path_budget_m=8.0)
    overridden = guarded_ur5e(UR5eConfig.declared(), envelope=other, rtde=FakeRTDE())
    assert overridden.shield.kernel.env.sha256() == other.sha256()


def test_configure_and_is_calibrated_delegate_to_the_follower():
    """The two members the adapter exists to supply, and the only two it may.

    They are not implemented here. They are forwarded to the follower, which
    is the object that knows whether a UR needs calibrating.
    """
    robot = guarded_ur5e(UR5eConfig.declared(), rtde=FakeRTDE())
    follower = robot.shield.robot

    calls: list[str] = []
    object.__setattr__(follower, "configure", lambda: calls.append("configure") or "sentinel-value")
    assert robot.configure() == "sentinel-value"
    assert calls == ["configure"]

    assert robot.is_calibrated is follower.is_calibrated is True


# ---- the adapter adds nothing ------------------------------------------------ #

class _RecordingShield:
    """A stand-in Shield that records what it was handed and returns markers.

    Nothing here resembles a real Shield's behaviour on purpose: the point is
    that the adapter cannot be doing anything to these values, because any
    clamp, validation, filter or reordering would have to touch them and
    identity would stop holding.
    """

    def __init__(self) -> None:
        self.robot = _RecordingFollower()
        self.kernel = object()
        self.last_verdict = object()
        self.received: list = []
        self.obs_marker: dict = {"joint_position": object()}
        self.action_marker: dict = {"returned": object()}
        self.features_marker: dict = {"features": object()}
        self.connected_marker = object()
        self.calls: list[str] = []

    def get_observation(self):
        self.calls.append("get_observation")
        return self.obs_marker

    def send_action(self, action):
        self.calls.append("send_action")
        self.received.append(action)
        return self.action_marker

    def connect(self):
        self.calls.append("connect")
        return "connect-return"

    def disconnect(self):
        self.calls.append("disconnect")
        return "disconnect-return"

    def calibrate(self):
        self.calls.append("calibrate")
        return "calibrate-return"

    @property
    def is_connected(self):
        return self.connected_marker

    @property
    def observation_features(self):
        return self.features_marker

    @property
    def action_features(self):
        return self.features_marker


class _RecordingFollower:
    def __init__(self) -> None:
        self.calls: list[str] = []

    def configure(self):
        self.calls.append("configure")
        return "follower-configure"

    @property
    def is_calibrated(self):
        return "follower-is-calibrated"


def test_the_adapter_is_a_pass_through():
    """Every member returns the wrapped object's own value, by identity.

    `is`, not `==`. A copy would compare equal and would still mean the
    adapter had touched the value. If this test ever needs `==` to pass,
    something started modifying what it forwards, and claim (A) would then
    be a claim about keystone.guarded rather than about sentinel's kernel.
    """
    shield = _RecordingShield()
    robot = GuardedUR5e(shield, UR5eConfig.declared())

    assert robot.get_observation() is shield.obs_marker

    action = {"joint_position": np.array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])}
    assert robot.send_action(action) is shield.action_marker
    assert shield.received == [action]
    assert shield.received[0] is action                       # not a copy
    assert shield.received[0]["joint_position"] is action["joint_position"]

    assert robot.observation_features is shield.features_marker
    assert robot.action_features is shield.features_marker
    assert robot.is_connected is shield.connected_marker
    assert robot.last_verdict is shield.last_verdict
    assert robot.connect() == "connect-return"
    assert robot.disconnect() == "disconnect-return"
    assert robot.calibrate() == "calibrate-return"

    # configure and is_calibrated are the two the Shield does not have, and
    # they go to the follower rather than being invented here.
    assert robot.configure() == "follower-configure"
    assert robot.is_calibrated == "follower-is-calibrated"
    assert shield.robot.calls == ["configure"]

    assert shield.calls == ["get_observation", "send_action", "connect", "disconnect",
                            "calibrate"]


def test_the_adapter_does_not_clamp_the_kernel_does():
    """A huge action is stopped, and it is stopped by the kernel, not here.

    keystone.follower has the matching test in the other direction: with no
    Shield in front of it, 1e9 reaches servoJ untouched, because the driver
    is not the safety layer. Composed, 1e9 must not reach servoJ, and what
    does reach it must be exactly what the kernel emitted.
    """
    fake = FakeRTDE(q0=EPISODE_Q0)
    robot = guarded_ur5e(UR5eConfig.declared(), rtde=fake)
    robot.connect()
    huge = np.full(6, 1e9)

    robot.get_observation()
    mark = len(fake.control.calls)
    robot.send_action({"joint_position": huge})

    sent = [c.args[0] for c in fake.control.calls_since(mark) if c.method == "servoJ"]
    assert len(sent) == 1
    assert not np.array_equal(sent[0], huge)
    assert np.array_equal(sent[0], robot.last_verdict.action.q)


# ---- claim (A): the test M-01 could not write -------------------------------- #

def test_the_attack_catalogue_is_not_empty():
    """Without this, an empty REGISTRY would make the parametrised test below
    pass by collecting nothing, which is the quietest way a suite can lie."""
    assert len(REGISTRY) == 15


@pytest.mark.parametrize("name", sorted(REGISTRY))
def test_no_attack_reaches_servoj_unauthorised(name):
    """Claim (A), exhaustively, for one attack from SENTINEL's catalogue.

    For every step: if the kernel emitted a command, exactly one servoJ call
    happened and its joint vector is bit-identical to the kernel's. If the
    kernel emitted none, nothing reached servoJ at all.

    Two further assertions are made about what reaches servoJ, because
    "bit-identical to what the kernel emitted" is only worth having if what
    the kernel emits is itself sound: every commanded value is finite, and
    every commanded value lies inside the envelope's joint limits. Those are
    sentinel.redteam's own definition of an escape, checked here through the
    real driver rather than through a simulated plant.

    Finally the guard each attack declares must be the guard that catches it,
    with one exemption whose premise is itself asserted at the end of this
    test. Without that, a composition that quietly swallowed every action
    would satisfy every assertion above by never commanding anything.
    """
    spec = REGISTRY[name]
    base = Envelope.ur5e_declared()
    # envelope_override applies to this attack's episode only, the same rule
    # sentinel.redteam.run_episode uses. slow_drift is the one attack that
    # declares one.
    env = base if spec.envelope_override is None else base.replace(**spec.envelope_override)
    rng = np.random.default_rng(0)
    attack = spec.build(env, rng)
    # start_q: an attack may declare where its episode starts. lever_sprint
    # does, because its ramp is legal in joint space from that posture only.
    q0 = EPISODE_Q0 if spec.start_q is None else np.asarray(spec.start_q, dtype=float)

    steps = STEPS_OVERRIDE.get(name, STEPS)

    clock = _Clock()
    fake = FakeRTDE(control_hz=UR5eConfig.declared().control_hz, q0=q0)
    if spec.freeze_state:
        # stale_replay declares freeze_state: the observation stops
        # advancing. Model that by stalling the RTDE stream itself, so
        # the controller serves a repeated sample and its timestamp
        # stands still. Freezing the host clock instead would prove
        # nothing now that the driver stamps from the controller.
        fake.freeze_stream()
    robot = guarded_ur5e(UR5eConfig.declared(), envelope=env, rtde=fake, clock=clock)
    robot.connect()

    rules_fired: set[str] = set()
    commanded = 0
    refused = 0
    try:
        for step in range(steps):
            obs = robot.get_observation()
            action = attack(_state_from(obs), step, clock.t)

            mark = len(fake.control.calls)
            robot.send_action(_payload(action))
            verdict = robot.last_verdict
            rules_fired.update(v.rule for v in verdict.violations)

            sent = [c.args[0] for c in fake.control.calls_since(mark) if c.method == "servoJ"]
            if verdict.action.q is None:
                refused += 1
                assert sent == [], (
                    f"{name} step {step}: a command reached servoJ when the kernel "
                    f"emitted none: {sent}")
            else:
                commanded += 1
                for q_sent in sent:
                    assert np.array_equal(q_sent, verdict.action.q), (
                        f"{name} step {step}: servoJ received {np.asarray(q_sent)} "
                        f"but the kernel emitted {verdict.action.q}")
                    q_sent = np.asarray(q_sent, dtype=float)
                    assert np.all(np.isfinite(q_sent)), (
                        f"{name} step {step}: a non-finite command reached servoJ: {q_sent}")
                    excursion = float(max(np.max(q_sent - env.q_max),
                                          np.max(env.q_min - q_sent), 0.0))
                    assert excursion == 0.0, (
                        f"{name} step {step}: a command outside the joint limits reached "
                        f"servoJ by {excursion:.6g} rad: {q_sent}")
                assert len(sent) == 1, (
                    f"{name} step {step}: expected exactly one servoJ call for one "
                    f"authorised command, saw {len(sent)}")

            # watchdog_starve declares skip_seconds and it is ticked on marked
            # steps, the same rule sentinel.redteam uses, because a skipped
            # step must actually exceed max_dt_s to reach the dt_max guard.
            extra = spec.skip_seconds if (spec.skip_seconds and step % 20 == 19) else 0.0
            clock.tick(DT + extra)
            if verdict.status is Status.STOP:
                robot.shield.kernel.rearm("composed episode continues after a trip")
    finally:
        robot.disconnect()

    assert commanded + refused == steps
    # The attack must actually be reaching the kernel. Without this, a
    # composition that silently dropped every action would pass every
    # assertion above by never commanding anything.
    assert rules_fired, f"{name}: no guard fired in {steps} steps; the attack never landed"

    if name == "force_grind":
        # The one attack whose declared guard cannot fire through this
        # composition, and the reason is this repository's fake rather than
        # M-01's kernel: keystone.rtde_fake.FakeReceive models no contact
        # physics and returns a zero wrench on every call, so force_max has
        # nothing to measure. Asserted rather than asserted-about, so the
        # exemption stands on a fact and not on a claim. Closing it needs a
        # contact model in the fake, or URSim, or an arm.
        assert fake.receive.getActualTCPForce() == [0.0] * 6
        assert "force_max" not in rules_fired
    else:
        assert spec.expect in rules_fired, (
            f"{name}: the guard it declares ({spec.expect}) never fired in {steps} "
            f"steps; fired instead: {sorted(rules_fired)}")


def test_non_finite_first_observation_sends_nothing_to_servoj():
    """The defect M-01 found and fixed, checked through the real composition.

    With no prior command and a non-finite observation the kernel has no
    trusted position to hold. The old behaviour fabricated np.zeros(6), whose
    flange sits 0.167200 m outside the declared box, and the Shield forwarded
    it. The fix is Action(q=None), meaning command no motion at all.

    Through this composition that has to mean the driver is never called, so
    servoJ is never reached, so the RTDE control interface never sees a
    command. Nothing weaker will do: not a zero pose, not the raw request,
    not a hold.
    """
    fake = FakeRTDE(q0=[np.nan, 0.0, 0.0, 0.0, 0.0, 0.0])
    robot = guarded_ur5e(UR5eConfig.declared(), rtde=fake)
    robot.connect()

    obs = robot.get_observation()
    assert not np.all(np.isfinite(obs["joint_position"]))

    mark = len(fake.control.calls)
    returned = robot.send_action({"joint_position": np.zeros(6)})

    assert robot.last_verdict.status is Status.STOP
    assert robot.last_verdict.action.q is None
    assert fake.control.calls_since(mark) == []
    assert fake.control.last_servoj_q is None
    # The return value must not echo the request back as though it had been
    # honoured. An empty dict is the honest answer to "what did the robot
    # receive", and the adapter forwards it unchanged.
    assert returned == {}


@pytest.mark.filterwarnings(
    # keystone.rtde_fake.FakeReceive.getActualQ computes `target - c.q`, and
    # with an infinite start that is inf - inf, which numpy reports as an
    # invalid operation and returns as nan. The observation is non-finite
    # either way, which is what this test is about, and the warning is a
    # property of the fake rather than of the composition under test.
    "ignore:invalid value encountered in subtract:RuntimeWarning")
def test_non_finite_first_observation_never_fabricates_a_zero_pose():
    """The specific fabricated value, named, so a regression is unambiguous."""
    fake = FakeRTDE(q0=[0.0, np.inf, 0.0, 0.0, 0.0, 0.0])
    robot = guarded_ur5e(UR5eConfig.declared(), rtde=fake)
    robot.connect()

    for _ in range(5):
        obs = robot.get_observation()
        assert not np.all(np.isfinite(obs["joint_position"]))
        robot.send_action({"joint_position": np.full(6, 0.3)})

    servoj = [c for c in fake.control.calls if c.method == "servoJ"]
    assert servoj == [], f"a command reached servoJ from a non-finite start: {servoj}"


# ---- the seam defect, found by composing, and now closed ---------------------- #

def test_the_driver_stamp_freezes_with_the_stream_so_stale_can_fire():
    """Regression test for a defect that only composition could expose.

    sentinel's staleness guard asks one question: is the driver's own
    observation stamp advancing. The driver originally answered it with
    time.perf_counter(), the host clock, which advances whether or not the
    RTDE stream does. A controller whose stream had frozen still produced a
    stamp marching forward, so `stale` could never fire through the real
    driver. Measured at the time: stale_replay through the composition fired
    qdd_max, qddd_max, tcp_box and tcp_speed, and never `stale`.

    It was invisible to M-01 by construction. M-01's fake driver could freeze
    its stamp on request; a real driver reading a host clock cannot. The
    probe could express the stall the shipped driver could not produce.

    The driver now stamps from RTDEReceiveInterface.getTimestamp(), the
    controller's own sample clock, which stands still exactly when the stream
    does. The kernel compares that stamp only against its own previous value
    and never against the kernel clock, so changing the epoch is safe. Both
    halves are asserted below: the stamp freezes, and the guard fires.
    """
    cfg = UR5eConfig.declared()

    # 1. A live stream advances the stamp.
    fake = FakeRTDE(control_hz=cfg.control_hz, q0=EPISODE_Q0)
    follower = UR5eFollower(cfg, rtde=fake)
    follower.connect()
    a = follower.get_observation()
    b = follower.get_observation()
    assert b["timestamp_monotonic"] > a["timestamp_monotonic"]

    # 2. A stalled stream freezes it, which the host clock would not do.
    fake.freeze_stream()
    c = follower.get_observation()
    d = follower.get_observation()
    assert d["timestamp_monotonic"] == c["timestamp_monotonic"], (
        "the driver stamp still advances while the RTDE stream is stalled; "
        "it is reading a host clock again")
    assert np.array_equal(c["joint_position"], d["joint_position"])

    # 3. And the guard that exists for this now actually fires through the
    #    whole composition, which is the property that was missing.
    env = Envelope.ur5e_declared()
    spec = REGISTRY["stale_replay"]
    attack = spec.build(env, np.random.default_rng(0))
    clock = _Clock()
    fake2 = FakeRTDE(control_hz=cfg.control_hz, q0=EPISODE_Q0)
    fake2.freeze_stream()
    robot = guarded_ur5e(cfg, envelope=env, rtde=fake2, clock=clock)
    robot.connect()

    rules: set[str] = set()
    for step in range(STEPS):
        obs = robot.get_observation()
        robot.send_action(_payload(attack(_state_from(obs), step, clock.t)))
        rules.update(v.rule for v in robot.last_verdict.violations)
        clock.tick()
        if robot.last_verdict.status is Status.STOP:
            robot.shield.kernel.rearm("composed episode continues after a trip")

    assert "stale" in rules, (
        f"the staleness guard did not fire against a stalled stream; "
        f"fired instead: {sorted(rules)}")
