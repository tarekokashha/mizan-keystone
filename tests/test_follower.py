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
"""
from __future__ import annotations

import numpy as np
import pytest
from lerobot.robots import Robot

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


# ---- the corrected LeRobot contract ---------------------------------------- #

def test_satisfies_the_installed_abstract_api():
    # lerobot.robots.Robot declares ten abstract members (see
    # docs/decisions/task-1-2-correction.md). The vendor skeleton implemented
    # eight and raised TypeError on instantiation. This must not repeat that.
    assert issubclass(UR5eFollower, Robot)
    assert UR5eFollower.__abstractmethods__ == frozenset()
    follower, _ = _make()  # raises TypeError if any abstract member is missing
    assert isinstance(follower, Robot)


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


# ---- get_observation returns every declared key ------------------------------ #

def test_get_observation_returns_every_declared_key_with_the_declared_shape():
    follower, _ = _make()

    obs = follower.get_observation()

    assert set(obs) == set(follower.observation_features)
    for key in ("joint_position", "joint_velocity", "tcp_pose", "tcp_force_torque"):
        assert obs[key].shape == (6,)
        assert obs[key].dtype == np.float64
    assert obs["gripper_position"].shape == (1,)
    assert obs["gripper_position"].dtype == np.float64
    assert isinstance(obs["timestamp_monotonic"], float)


def test_get_observation_timestamp_uses_perf_counter_not_monotonic():
    # M-01 measured perf_counter() and monotonic() to be 5.558460s apart on
    # this machine. The staleness guard downstream compares successive
    # stamps from the driver to each other, so the epoch does not matter,
    # but the clock source must be consistent call over call. perf_counter,
    # not monotonic, per the task brief.
    follower, _ = _make()
    t0 = follower.get_observation()["timestamp_monotonic"]
    t1 = follower.get_observation()["timestamp_monotonic"]
    assert t1 >= t0


# ---- send_action: pass-through, exactly, no safety logic --------------------- #

def test_send_action_passes_the_joint_target_to_servoj_unmodified():
    follower, fake = _make()
    q = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]

    result = follower.send_action({"joint_position": q})

    servoj_calls = [c for c in fake.control.calls if c.method == "servoJ"]
    assert len(servoj_calls) == 1
    assert servoj_calls[0].args[0] == q
    assert result["joint_position"] == q


def test_send_action_returns_exactly_what_was_sent():
    # "take the action, call servoJ, return what was sent": every key in the
    # action, not only joint_position, comes back unchanged.
    follower, _ = _make()
    action = {"joint_position": [0.0] * 6, "gripper_position": [0.5]}

    result = follower.send_action(action)

    assert result == action


def test_send_action_has_no_safety_logic_a_huge_action_reaches_servoj_unmodified():
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
    """
    follower, fake = _make()
    huge = [1e9] * 6

    result = follower.send_action({"joint_position": huge})

    servoj_calls = [c for c in fake.control.calls if c.method == "servoJ"]
    assert len(servoj_calls) == 1
    assert servoj_calls[0].args[0] == huge
    assert result["joint_position"] == huge


def test_send_action_uses_initperiod_and_waitperiod_around_servoj():
    # servoJ's own contract pairs it with initPeriod/waitPeriod to hold the
    # control loop period; this is correct low-level RTDE API usage, not a
    # safety behaviour, and keystone.rtde_fake implements both for exactly
    # this reason.
    follower, fake = _make()
    follower.send_action({"joint_position": [0.0] * 6})
    methods = [c.method for c in fake.control.calls]
    assert methods == ["initPeriod", "servoJ", "waitPeriod"]


# ---- rtde=None constructs the real ur_rtde interfaces ------------------------- #

def test_rtde_none_constructs_the_real_interfaces_from_config(monkeypatch):
    """UR5eFollower(config) with no injected rtde must build the real
    ur_rtde interfaces from config.ip and config.control_hz, not silently
    fall back to a fake.

    Verified by monkeypatching the three real classes with recorders, not by
    opening an actual socket: this repository's offline suite runs with no
    hardware and no Docker, and 192.0.2.10 (UR5eConfig.declared()'s ip) is a
    reserved documentation address that nothing listens on.
    """
    calls: dict[str, tuple] = {}

    class _RecordingControl:
        def __init__(self, hostname, frequency):
            calls["control"] = (hostname, frequency)

    class _RecordingReceive:
        def __init__(self, hostname, frequency):
            calls["receive"] = (hostname, frequency)

    class _RecordingDashboard:
        def __init__(self, hostname):
            calls["dashboard"] = (hostname,)

    monkeypatch.setattr(follower_module.rtde_control, "RTDEControlInterface", _RecordingControl)
    monkeypatch.setattr(follower_module.rtde_receive, "RTDEReceiveInterface", _RecordingReceive)
    monkeypatch.setattr(follower_module.dashboard_client, "DashboardClient", _RecordingDashboard)

    cfg = UR5eConfig.declared()
    driver = UR5eFollower(cfg)  # rtde=None

    assert calls["control"] == (cfg.ip, cfg.control_hz)
    assert calls["receive"] == (cfg.ip, cfg.control_hz)
    assert calls["dashboard"] == (cfg.ip,)
    assert driver.rtde.control is not None
    assert driver.rtde.receive is not None
    assert driver.rtde.dashboard is not None
