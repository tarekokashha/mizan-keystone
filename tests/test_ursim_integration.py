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
