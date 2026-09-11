"""keystone.follower: UR5eFollower, a real LeRobot plugin over RTDE.

This module implements lerobot.robots.Robot's abstract API against the
installed lerobot 0.4.4, measured directly rather than copied from the
vendor kit's skeleton. lerobot.robots.Robot declares ten abstract members:
action_features, calibrate, configure, connect, disconnect, get_observation,
is_calibrated, is_connected, observation_features, send_action. The
skeleton implemented eight of them (missing configure and is_calibrated)
and could not be instantiated: TypeError on construction, before any RTDE
call. See docs/decisions/task-1-2-correction.md, Finding 1. UR5eFollower
implements all ten.

This driver carries no safety logic, on purpose. The vendor skeleton's
send_action had six defects, all of them safety logic: a velocity clamp
wrong by the ratio of two rates, no position limits, an unsound force
check, no finiteness validation, no watchdog, and no cumulative guard (see
docs/superpowers/specs/2026-09-11-m03-keystone-design.md, section 4, "The
driver does no safety, on purpose"). None of that is fixed here. It is
removed. send_action does exactly three things: take the action, call
servoJ, return what was sent. No clamp, no limit, no finiteness check
beyond shape, no watchdog, no cumulative guard. A driver with its own
half-correct clamp is worse than one with none, because it invites the
reader to trust it. Safety is sentinel.Shield's job (M-01 SENTINEL),
already built, reviewed and measured; Task 4 composes it in front of this
driver so nothing reaches servoJ the Shield did not authorise. If you find
yourself adding a guard here, stop: it belongs in sentinel.Envelope or the
Shield, not in this module.

One thing this driver does carry, and it is not safety logic: the
connection contract the base class states. lerobot.robots.Robot's own
docstring for is_connected says that when it is False, calling
get_observation or send_action "should raise an error". They did not; a
send_action after disconnect() issued servoJ on a control interface the
driver had already torn down. get_observation and send_action now carry
lerobot.utils.decorators.check_if_not_connected, the vendor's own check.
That is transport state, and it reads is_connected and nothing else. It
does not look at the action, so the claim above is untouched: no value in
an action is clamped, validated or filtered here.

`rtde` injects the deterministic fakes from keystone.rtde_fake, so the
whole class is testable offline with no hardware and no Docker. When rtde
is None, the real ur_rtde interfaces are constructed from config.ip and
config.control_hz, bundled the same way keystone.rtde_fake.FakeRTDE bundles
its fakes (.control, .receive, .dashboard), so the rest of this class does
not need to know which one it was given. They are constructed in connect(),
not in __init__: see the comment there for what the installed ur_rtde does
on construction.
"""
from __future__ import annotations

from typing import Any

import dashboard_client
import numpy as np
import rtde_control
import rtde_receive
from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots import Robot
from lerobot.utils.decorators import check_if_not_connected

from keystone.config import UR5eConfig

N_JOINTS = 6


class _RTDEBundle:
    """The three real ur_rtde interfaces, bundled under .control, .receive
    and .dashboard, the same attribute names keystone.rtde_fake.FakeRTDE
    uses for its fakes. UR5eFollower addresses self.rtde.control (and
    .receive, .dashboard) uniformly, whether rtde is this bundle or a
    FakeRTDE injected for testing.
    """

    def __init__(self, control: Any, receive: Any, dashboard: Any) -> None:
        self.control = control
        self.receive = receive
        self.dashboard = dashboard


class UR5eFollower(Robot):
    """A UR5e follower driven over RTDE. No safety logic; see the module
    docstring.
    """

    config_class = UR5eConfig
    name = "ur5e"

    def __init__(self, config: UR5eConfig, rtde: Any | None = None) -> None:
        super().__init__(config)
        self.config = config
        # rtde is stored as given and NOTHING is opened here. When it is
        # None the real interfaces are built in connect(), not in this
        # constructor, because constructing them is not inert: the installed
        # ur_rtde 1.6.5 declares
        #
        #   __init__(self, hostname, frequency=-1.0,
        #            flags=<Flags.FLAG_UPLOAD_SCRIPT: 1>, ...)
        #
        # measured from the pybind11 signature, so merely constructing a
        # UR5eFollower would reach the address in config.ip and upload a
        # control script before any caller had said connect(). The injection
        # design is unchanged: an injected rtde is used exactly as handed
        # over, and self.rtde is still the single attribute the rest of this
        # class addresses.
        self.rtde = rtde
        self._connected = False

    # ---- LeRobot feature contracts ---------------------------------------- #
    @property
    def observation_features(self) -> dict[str, type | tuple]:
        return {
            "joint_position": (N_JOINTS,),
            "joint_velocity": (N_JOINTS,),
            "tcp_pose": (N_JOINTS,),
            "tcp_force_torque": (N_JOINTS,),
            "gripper_position": (1,),
            "timestamp_monotonic": float,
        }

    @property
    def action_features(self) -> dict[str, type | tuple]:
        return {"joint_position": (N_JOINTS,), "gripper_position": (1,)}

    # ---- lifecycle ----------------------------------------------------------- #
    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self, calibrate: bool = True) -> None:
        # calibrate is accepted for signature compatibility with callers
        # that pass it (as real LeRobot scripts do); it has no effect,
        # because is_calibrated is always True below.
        if self.rtde is None:
            # The three real interfaces, built in the same order and at the
            # same point in the sequence they were built before: still ahead
            # of every dashboard call, so nothing about what this driver does
            # to an arm changes, only when the sockets open. See __init__.
            self.rtde = _RTDEBundle(
                control=rtde_control.RTDEControlInterface(
                    self.config.ip, frequency=self.config.control_hz),
                receive=rtde_receive.RTDEReceiveInterface(
                    self.config.ip, frequency=self.config.control_hz),
                dashboard=dashboard_client.DashboardClient(self.config.ip),
            )
        self.rtde.dashboard.connect()
        self.rtde.dashboard.powerOn()
        self.rtde.dashboard.brakeRelease()
        self._connected = True
        self.configure()

    @property
    def is_calibrated(self) -> bool:
        # UR arms are factory calibrated. Robot's own docstring for this
        # property says it "should be always True if not applicable", and
        # calibration does not apply to this driver.
        return True

    def calibrate(self) -> None:
        # A no-op for the same reason is_calibrated is always True. Robot
        # requires the hook to exist; there is nothing for it to do.
        return None

    def configure(self) -> None:
        # Nothing to configure. servoJ's lookahead and gain are supplied
        # per call from config in send_action, not set once here. Robot
        # requires the hook to exist.
        return None

    # ---- I/O ------------------------------------------------------------------ #
    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        return {
            "joint_position": np.asarray(self.rtde.receive.getActualQ(), dtype=np.float64),
            "joint_velocity": np.asarray(self.rtde.receive.getActualQd(), dtype=np.float64),
            "tcp_pose": np.asarray(self.rtde.receive.getActualTCPPose(), dtype=np.float64),
            "tcp_force_torque": np.asarray(self.rtde.receive.getActualTCPForce(), dtype=np.float64),
            # No gripper hardware is driven in this task (see
            # LIMITATIONS.md): a fixed placeholder, not a measurement.
            # Never a real-robot number.
            "gripper_position": np.zeros(1, dtype=np.float64),
            # The CONTROLLER's sample clock, not this host's. The kernel's
            # staleness guard asks whether the driver's stamp is advancing,
            # and compares it only against its own previous value, never
            # against the kernel clock, so the epoch does not matter but the
            # source does. Stamping this with time.perf_counter() made that
            # guard blind: the host clock advances whether or not the RTDE
            # stream does, so a stalled controller still looked fresh and
            # "stale" could never fire through the real driver. Measured and
            # recorded in docs/decisions/task-4-staleness-is-blind.md.
            "timestamp_monotonic": float(self.rtde.receive.getTimestamp()),
        }

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        # The decorator is transport state, not safety: Robot.is_connected's
        # own docstring requires get_observation and send_action to raise
        # when is_connected is False, and lerobot.utils.decorators supplies
        # the check, so this is the vendor's contract honoured rather than a
        # rule invented here. It reads is_connected and nothing else; it does
        # not look at the action at all. Everything below is unchanged.
        #
        # No clamp, no limit, no finiteness check, no watchdog, no
        # cumulative guard. This is not an omission. See the module
        # docstring and
        # test_send_action_has_no_safety_logic_a_huge_action_reaches_servoj_unmodified
        # in tests/test_follower.py. initPeriod/waitPeriod bracket servoJ
        # because that is servoJ's own low-level API contract for holding
        # a control period; it is not a safety behaviour.
        q = action["joint_position"]
        t_start = self.rtde.control.initPeriod()
        self.rtde.control.servoJ(
            q, 0.0, 0.0, 1.0 / self.config.control_hz,
            self.config.servo_lookahead, self.config.servo_gain,
        )
        self.rtde.control.waitPeriod(t_start)
        return dict(action)

    def disconnect(self) -> None:
        if self.rtde is None:
            # connect() never ran, so no interface exists to stop, and
            # building one here purely to close it would open the sockets
            # this driver just stopped opening in __init__.
            self._connected = False
            return
        try:
            self.rtde.control.servoStop()
            self.rtde.control.stopScript()
        finally:
            self.rtde.control.disconnect()
        self.rtde.receive.disconnect()
        self.rtde.dashboard.disconnect()
        self._connected = False
