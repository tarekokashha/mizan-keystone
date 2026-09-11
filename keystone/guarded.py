"""keystone.guarded: M-01 SENTINEL's Shield composed with this repository's driver.

This module closes M-01 SENTINEL's largest stated limitation, that the Shield
had never been composed with the driver it exists to wrap. The composition is:

    policy / teleop / LeRobot
              |
              v
        GuardedUR5e     (this module, a LeRobot plugin, NO safety logic)
              |
              v
        Shield          (M-01 SENTINEL, installed as a dependency)
              |
              v
        UR5eFollower    (keystone.follower, NO safety logic)
              |
              v
        RTDEControlInterface  |  FakeRTDE

Why an adapter exists at all. The plan gave this task the interface
`guarded_ur5e(...) -> Shield`. That does not work, and the reason is the
same defect this whole programme was chartered to fix one layer down:
sentinel.shield.Shield implements eight of lerobot.robots.Robot's ten
abstract members and does not inherit from Robot. The two it lacks,
configure and is_calibrated, are exactly the two the vendor kit's skeleton
lacked. Measured, not read; see
docs/decisions/task-4-shield-is-not-a-lerobot-plugin.md. A Shield cannot be
handed to lerobot-record, and isinstance(shield, Robot) is False.

So guarded_ur5e returns a GuardedUR5e: a thin Robot subclass that supplies
those two members by delegation and adds nothing else.

GuardedUR5e adds no safety logic whatsoever. No clamp, no validation, no
filtering, no reordering, no defensive copy. Every member is either a
forward to the Shield or, for the two the Shield does not have, a forward to
the follower underneath it. That is not laziness, it is the whole point: the
claim under test is that every value reaching servoJ is a value the SENTINEL
kernel emitted, unmodified. If this adapter touched a value on the way past,
that claim would become a claim about this file rather than about the
kernel, and it would be worth nothing. tests/test_guarded.py proves the
pass-through by object identity rather than by equality, because a defensive
copy would compare equal and would still mean this file had touched
something.

If you are reading this because you are about to add a check here: it belongs
in sentinel.Envelope or sentinel.SafetyKernel, where it will be measured
against the attack catalogue. Not here.
"""
from __future__ import annotations

from typing import Any, Callable

from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots import Robot
from sentinel.envelope import Envelope
from sentinel.journal import Journal
from sentinel.kernel import SafetyKernel
from sentinel.shield import Shield

from keystone.config import UR5eConfig
from keystone.follower import UR5eFollower


class GuardedUR5e(Robot):
    """A LeRobot plugin wrapping a Shield. Delegation only.

    config_class and name are set because Robot requires both in every
    subclass. name is "ur5e", the same as UR5eFollower's, deliberately: it is
    the same robot type and the same arm, and Robot derives its calibration
    directory from it. The guarded and unguarded stacks addressing one
    calibration record is correct. Nothing in lerobot dispatches on Robot.name
    (make_robot_from_config dispatches on the config's registered type), so
    sharing it collides with nothing.
    """

    config_class = UR5eConfig
    name = "ur5e"

    def __init__(self, shield: Shield, config: UR5eConfig) -> None:
        super().__init__(config)
        self.config = config
        self.shield = shield

    # ---- the two members sentinel.shield.Shield does not have -------------- #
    #
    # These are the entire reason this class exists. Neither is implemented
    # here; both are forwarded to the follower, which is the object that
    # knows whether a UR needs calibrating. Inventing an answer here would
    # make this adapter a participant rather than a pass-through.
    def configure(self) -> Any:
        return self.shield.robot.configure()

    @property
    def is_calibrated(self) -> Any:
        return self.shield.robot.is_calibrated

    # ---- everything else is the Shield's, unmodified ----------------------- #
    def get_observation(self) -> RobotObservation:
        return self.shield.get_observation()

    def send_action(self, action: RobotAction) -> RobotAction:
        # The action goes to the Shield exactly as received, and the Shield's
        # answer comes back exactly as returned. No clamp, no validation, no
        # filtering, no reordering, no copy. When the kernel refuses to
        # command motion the Shield returns an empty dict and never calls the
        # driver at all; that empty dict is forwarded too, rather than being
        # dressed up as a fulfilled request.
        return self.shield.send_action(action)

    def connect(self, calibrate: bool = True) -> Any:
        # calibrate is accepted because Robot.connect declares it and real
        # LeRobot scripts pass it. Shield.connect takes no such argument and
        # the follower's own connect ignores it (is_calibrated is always
        # True), so there is nothing to forward it to.
        return self.shield.connect()

    def disconnect(self) -> Any:
        return self.shield.disconnect()

    def calibrate(self) -> Any:
        return self.shield.calibrate()

    @property
    def is_connected(self) -> Any:
        return self.shield.is_connected

    @property
    def observation_features(self) -> Any:
        return self.shield.observation_features

    @property
    def action_features(self) -> Any:
        return self.shield.action_features

    @property
    def last_verdict(self) -> Any:
        """The kernel's decision about the most recent action.

        Read-only forward of Shield.last_verdict. This is what
        tests/test_guarded.py compares against what reached servoJ, and it is
        what an operator reads to find out why the arm did not move.
        """
        return self.shield.last_verdict


def guarded_ur5e(
    config: UR5eConfig,
    envelope: Envelope | None = None,
    rtde: Any | None = None,
    *,
    journal: Journal | None = None,
    clock: Callable[[], float] | None = None,
) -> GuardedUR5e:
    """Build the guarded stack from one config.

    Args:
        config: the driver's connection and servo parameters. Carries no
            safety limit; those live in the envelope, and having two sources
            of truth for a safety limit is worse than having one.
        envelope: the declared safety envelope. Defaults to
            Envelope.ur5e_declared(), M-01's own declaration.
        rtde: injected RTDE interfaces, normally a keystone.rtde_fake.FakeRTDE.
            None means the follower builds the real ur_rtde interfaces from
            config.ip.
        journal: optional hash-chained log, forwarded to the Shield untouched.
        clock: optional clock for the kernel, forwarded to SafetyKernel. When
            omitted the kernel keeps its own default, which is
            time.perf_counter and is deliberately not restated here: the
            kernel owns that choice and documents why (on Windows
            time.monotonic has 15.625 ms resolution, half a control period at
            30 Hz, and the shaping chain divides by dt three times). Injection
            exists because dt-dependent guards cannot be exercised
            deterministically against a wall clock; SafetyKernel already
            provides the parameter for exactly that reason.

    Returns:
        A GuardedUR5e, which is a lerobot.robots.Robot. Not a Shield: a
        Shield is not a Robot, which is the finding recorded in
        docs/decisions/task-4-shield-is-not-a-lerobot-plugin.md.
    """
    env = Envelope.ur5e_declared() if envelope is None else envelope
    kernel = SafetyKernel(env) if clock is None else SafetyKernel(env, clock=clock)
    follower = UR5eFollower(config, rtde=rtde)
    return GuardedUR5e(Shield(follower, kernel, journal=journal), config)
