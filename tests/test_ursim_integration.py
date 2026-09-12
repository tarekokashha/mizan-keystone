"""The URSim integration, which closes the unmeasured half of claim (B).

Separate from tests/test_ursim.py on purpose. That file tests the skip logic
and runs everywhere in milliseconds. This one pulls a multi-gigabyte image
and boots a robot controller, so it is opt in: it runs only when Docker
answers AND KEYSTONE_URSIM is set, which the ursim CI job does and nothing
else does.

It asserts structural facts and measures timing. It asserts nothing about
the timing, because PROTOCOL.md section 6 commits this programme to
reporting rates rather than gating on them, and that commitment does not
weaken just because the numbers finally come from a controller instead of a
fake. If this file ever grows a threshold, the same argument applies as in
tests/test_timing.py: argue for it in PROTOCOL.md first.
"""
from __future__ import annotations

import os
import time

import numpy as np
import pytest

from keystone.ursim import (
    BOOT_TIMEOUT_S,
    URSIM_DASHBOARD_PORT,
    URSIM_IMAGE,
    URSIM_RTDE_PORT,
    container_logs,
    docker_status,
    pull_image,
    start_container,
    stop_container,
    power_on,
    wait_for_rtde,
    wait_for_tcp,
)

_OPTED_IN = os.environ.get("KEYSTONE_URSIM", "") not in ("", "0", "false")
_STATUS = docker_status() if _OPTED_IN else None

pytestmark = pytest.mark.skipif(
    not _OPTED_IN or (_STATUS is not None and not _STATUS.available),
    reason=("KEYSTONE_URSIM is not set" if not _OPTED_IN
            else f"URSim unavailable: {_STATUS.reason if _STATUS else 'unknown'}"),
)

HOST = "127.0.0.1"
SAMPLES = 2000


@pytest.fixture(scope="module")
def ursim():
    """A booted URSim, or a failure that says exactly how far it got.

    Every stage reports what it observed rather than raising a bare error,
    because "URSim did not work" is not a useful thing to read in a CI log.
    """
    pulled = pull_image()
    if not pulled.available:
        pytest.fail(f"could not pull {URSIM_IMAGE}: {pulled.reason}")

    handle, reason = start_container()
    if handle is None:
        pytest.fail(f"could not start URSim: {reason}")

    try:
        if not wait_for_tcp(HOST, URSIM_DASHBOARD_PORT, BOOT_TIMEOUT_S):
            pytest.fail(
                f"URSim dashboard port {URSIM_DASHBOARD_PORT} never opened within "
                f"{BOOT_TIMEOUT_S:g}s. Container logs:\n{container_logs()}")
        if not wait_for_tcp(HOST, URSIM_RTDE_PORT, BOOT_TIMEOUT_S):
            pytest.fail(
                f"URSim RTDE port {URSIM_RTDE_PORT} never opened within "
                f"{BOOT_TIMEOUT_S:g}s. Container logs:\n{container_logs()}")
        # The port listening is not the controller being ready. Measured on a
        # runner: the socket accepts well before RTDE will open a session, and
        # the interface then fails with "read: Connection reset by peer". So
        # wait for the thing that is actually needed.
        ready, why = wait_for_rtde(HOST, BOOT_TIMEOUT_S)
        if not ready:
            pytest.fail(
                f"URSim never served RTDE: {why}\nContainer logs:\n{container_logs()}")
        print(f"\nURSim ready: {why}")
        # A connected RTDE session is still not a running robot. URSim boots
        # powered off, and measured on a runner its timestamp then stood
        # still across 2000 consecutive reads. That is precisely the stalled
        # stream the kernel's staleness guard exists to catch, arriving here
        # as a real controller state rather than a simulated one.
        powered, mode = power_on(HOST, BOOT_TIMEOUT_S)
        if not powered:
            pytest.fail(f"URSim never powered on: {mode}\nContainer logs:\n{container_logs()}")
        print(f"URSim powered: {mode}")
        yield handle
    finally:
        stop_container()


def test_the_image_name_and_ports_are_now_verified(ursim):
    """They were marked unverified because the engine never answered on the
    development host. Reaching this assertion means a container built from
    URSIM_IMAGE booted and opened both declared ports, so the constants are
    measured rather than guessed. keystone/ursim.py's provenance note should
    be updated when this first passes.
    """
    assert ursim.image == URSIM_IMAGE
    assert ursim.container_id


def test_rtde_receive_connects_to_ursim_and_reports_six_finite_joints(ursim):
    import rtde_receive

    rtde = rtde_receive.RTDEReceiveInterface(HOST)
    try:
        assert rtde.isConnected()
        q = np.asarray(rtde.getActualQ(), dtype=float)
        assert q.shape == (6,), f"expected six joints, got {q.shape}"
        assert np.all(np.isfinite(q)), f"URSim reported a non-finite joint: {q}"
    finally:
        rtde.disconnect()


def test_the_controller_clock_advances_and_its_rate_is_reported(ursim):
    """The measurement the fake could never provide.

    keystone.follower stamps observations from getTimestamp(), the
    controller's own sample clock, precisely because the host clock kept
    advancing while the stream stalled. Here that clock comes from a real
    controller for the first time.

    Reported, not gated.
    """
    import rtde_receive

    rtde = rtde_receive.RTDEReceiveInterface(HOST)
    try:
        stamps = np.empty(SAMPLES, dtype=np.float64)
        wall = np.empty(SAMPLES, dtype=np.float64)
        for i in range(SAMPLES):
            stamps[i] = rtde.getTimestamp()
            wall[i] = time.perf_counter()

        advanced = np.diff(stamps)
        moved = advanced[advanced > 0]
        assert moved.size > 0, (
            "the controller timestamp never advanced across "
            f"{SAMPLES} reads, so the RTDE stream is not live")

        elapsed = wall[-1] - wall[0]
        wall_us = np.diff(wall) * 1e6
        print(
            "\nURSim RTDE, measured and not asserted:"
            f"\n  samples              : {SAMPLES}"
            f"\n  wall elapsed s       : {elapsed:.6f}"
            f"\n  read rate Hz         : {SAMPLES / elapsed if elapsed > 0 else float('inf'):.1f}"
            f"\n  controller dt p50 s  : {float(np.median(moved)):.6f}"
            f"\n  read gap p50 us      : {float(np.percentile(wall_us, 50)):.2f}"
            f"\n  read gap p99 us      : {float(np.percentile(wall_us, 99)):.2f}"
            f"\n  read gap p99.9 us    : {float(np.percentile(wall_us, 99.9)):.2f}"
            f"\n  read gap max us      : {float(wall_us.max()):.2f}"
        )
    finally:
        rtde.disconnect()


def test_the_real_driver_reads_a_real_controller(ursim):
    """The production path, end to end, for the first time.

    Everything else in this repository drives UR5eFollower through the
    deterministic fake. Here it constructs the genuine ur_rtde interfaces
    from its own config and talks to a real UR controller: the same
    connect(), the same get_observation(), the same getTimestamp() the
    staleness fix introduced.

    ip is overridden to the container because the declared config carries a
    TEST-NET-1 documentation address, deliberately, so that this repository
    never ships a routable robot address.
    """
    from keystone.config import UR5eConfig
    from keystone.follower import UR5eFollower

    cfg = UR5eConfig.declared().replace(ip=HOST)
    robot = UR5eFollower(cfg)
    robot.connect()
    try:
        assert robot.is_connected

        first = robot.get_observation()
        assert first["joint_position"].shape == (6,)
        assert np.all(np.isfinite(first["joint_position"]))
        assert np.all(np.isfinite(first["tcp_pose"]))

        # The controller clock, through the driver, against real hardware
        # simulation. This is the property the staleness fix turns on.
        stamps = []
        deadline = time.perf_counter() + 30.0
        while len(stamps) < 200 and time.perf_counter() < deadline:
            stamps.append(robot.get_observation()["timestamp_monotonic"])
        arr = np.asarray(stamps, dtype=float)
        advanced = np.diff(arr)
        assert np.any(advanced > 0), (
            "the driver's stamp never advanced against a running controller, "
            f"so get_observation is not reading a live stream: {arr[:5]}")
        assert np.all(advanced >= 0), (
            f"the controller stamp went backwards, which the kernel treats as "
            f"a driver fault: min delta {advanced.min()}")
        print(
            "\nUR5eFollower against URSim, measured and not asserted:"
            f"\n  observations         : {len(stamps)}"
            f"\n  controller dt p50 s  : {float(np.median(advanced[advanced > 0])):.6f}"
            f"\n  stamp span s         : {float(arr[-1] - arr[0]):.6f}"
        )
    finally:
        robot.disconnect()
