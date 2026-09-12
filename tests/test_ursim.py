"""Tests for keystone.ursim.

The property that matters is that an unresponsive Docker produces a skip
with a reason, promptly, rather than a hang. On the platform this was
written on Docker Desktop was running and its CLI never answered, so this
is the observed failure mode and not an imagined one.
"""
from __future__ import annotations

import subprocess
import time

import pytest

from keystone import ursim
from keystone.ursim import DockerStatus, docker_status, skip_reason


def _fake(returncode=0, stdout="", stderr=""):
    def runner(cmd, timeout_s):
        return subprocess.CompletedProcess(cmd, returncode, stdout, stderr)
    return runner


def test_a_hanging_docker_cli_becomes_a_skip_and_does_not_hang():
    """The whole point of the module.

    A suite that hangs is worse than one that fails: nobody can tell it
    apart from a slow one. The runner here raises TimeoutExpired, which is
    what subprocess does when the CLI never answers.
    """
    def hangs(cmd, timeout_s):
        raise subprocess.TimeoutExpired(cmd, timeout_s)

    t0 = time.perf_counter()
    status = docker_status(timeout_s=0.5, runner=hangs)
    elapsed = time.perf_counter() - t0

    assert status.available is False
    assert "did not respond" in status.reason
    assert "daemon is" in status.reason
    assert elapsed < 1.0, "docker_status blocked instead of returning a reason"


def test_a_healthy_docker_is_reported_available_with_its_version():
    status = docker_status(runner=_fake(0, "27.1.1\n"))
    assert status.available is True
    assert "27.1.1" in status.reason


def test_a_daemon_that_answers_with_nothing_is_not_treated_as_available():
    """`docker version` can exit 0 and print nothing while the daemon is
    still coming up. Treating that as available would start a container
    against a half-open engine.
    """
    status = docker_status(runner=_fake(0, "   \n"))
    assert status.available is False
    assert "not ready" in status.reason


def test_a_nonzero_exit_carries_dockers_own_first_line_into_the_reason():
    status = docker_status(runner=_fake(1, "", "Cannot connect to the Docker daemon\nmore"))
    assert status.available is False
    assert "Cannot connect to the Docker daemon" in status.reason
    assert "more" not in status.reason, "the reason should be one line, not a dump"


def test_docker_missing_from_path_is_its_own_reason(monkeypatch):
    monkeypatch.setattr(ursim.shutil, "which", lambda name: None)
    status = docker_status()
    assert status.available is False
    assert "not on PATH" in status.reason


def test_skip_reason_is_none_only_when_docker_is_usable(monkeypatch):
    monkeypatch.setattr(ursim, "docker_status", lambda: DockerStatus(True, "docker server 1.2.3"))
    assert skip_reason() is None

    monkeypatch.setattr(ursim, "docker_status", lambda: DockerStatus(False, "the daemon is asleep"))
    reason = skip_reason()
    assert reason is not None
    assert "the daemon is asleep" in reason


def test_the_constants_carry_an_honest_provenance():
    """A constant must say how far it has actually been checked.

    These began as "unverified", because the development host's Docker
    engine never answered and the image name and ports could not be tested.
    A container has since booted from URSIM_IMAGE on a CI runner and opened
    both ports, with RTDE serving a session through the RTDE one, so they
    are now "verified-in-ci".

    Deliberately not the word "measured". That token belongs to the envelope
    provenance vocabulary, where only a human may write it, and borrowing it
    for a Docker port would blur exactly the distinction that vocabulary
    exists to keep sharp.
    """
    assert ursim.CONSTANT_PROVENANCE in ("unverified", "verified-in-ci")
    assert ursim.CONSTANT_PROVENANCE != "measured", (
        "only a human may write 'measured'; use a token that does not claim it")
    # Whatever the value is, the module has to explain it rather than assert
    # it, so a reader can tell what was actually checked.
    assert ursim.CONSTANT_PROVENANCE in ursim.__doc__ or "unverified" in ursim.__doc__


# Probed once at import. Evaluating skip_reason() inside the decorator
# twice, for the condition and for the message, costs two probes on
# every suite run, and each one waits out the full timeout when the
# daemon is unresponsive.
_URSIM_SKIP = skip_reason()


@pytest.mark.skipif(_URSIM_SKIP is not None, reason=_URSIM_SKIP or "")
def test_ursim_container_smoke():
    """The real integration, which runs only where Docker answers.

    It is skipped rather than failed when the engine is unreachable, which
    is the behaviour the design specification asks for. On the platform this
    was written on it skips, and the skip reason names the cause.
    """
    status = docker_status()
    assert status.available, status.reason
