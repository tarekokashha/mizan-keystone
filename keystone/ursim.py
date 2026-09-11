"""keystone.ursim: URSim lifecycle, and a clean skip when Docker is absent.

The design specification requires URSim integration that is "skipped with a
clear reason when Docker is unavailable rather than failing". This module's
hardest requirement is not starting a container. It is never hanging.

That is not hypothetical. On the platform this was written on, Docker
Desktop's processes were running while `docker version`, `docker ps` and
`docker info` each returned no output, no error, and had to be killed by a
timeout. A naive subprocess.run(["docker", "info"]) in a test suite would
block forever against that, and a suite that hangs is worse than one that
fails, because nobody can tell it apart from a slow one. Every docker
invocation here therefore carries a timeout, and a timeout is treated as
"Docker unavailable" with that reason recorded, not as an error to retry.

PROVENANCE OF THE CONSTANTS BELOW. The image name and ports are
configuration defaults, not measurements. They could not be verified on
this host because the Docker engine never answered, so they are marked
unverified and this module does not claim otherwise. Nothing here is a
robot number, and nothing here was filled in to look complete: if a value
was not checked, it says so.
"""
from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass
from typing import Callable, Sequence

# Seconds. Short on purpose: this is a liveness probe, not a pull. A healthy
# daemon answers `docker version` in well under a second, and an unhealthy
# one never answers at all, so waiting longer only lengthens the hang.
PROBE_TIMEOUT_S = 5.0

# Unverified configuration defaults. See the module docstring.
URSIM_IMAGE = "universalrobots/ursim_e-series"
URSIM_DASHBOARD_PORT = 29999
URSIM_RTDE_PORT = 30004
CONSTANT_PROVENANCE = "unverified"


@dataclass(frozen=True)
class DockerStatus:
    """Whether Docker can be used, and a reason fit to print in a skip."""

    available: bool
    reason: str


def _run(cmd: Sequence[str], timeout_s: float) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)


def docker_status(timeout_s: float = PROBE_TIMEOUT_S,
                  runner: Callable[..., subprocess.CompletedProcess] = _run) -> DockerStatus:
    """Probe Docker without ever blocking longer than `timeout_s`.

    `runner` is injected so the hang path can be tested without needing a
    hung Docker, which is not a state a test can reliably create.
    """
    if shutil.which("docker") is None:
        return DockerStatus(False, "docker is not on PATH")

    try:
        proc = runner(["docker", "version", "--format", "{{.Server.Version}}"], timeout_s)
    except subprocess.TimeoutExpired:
        return DockerStatus(
            False,
            f"the docker CLI did not respond within {timeout_s:g}s; the daemon is "
            f"not serving its API (its processes may still be running)")
    except OSError as exc:
        return DockerStatus(False, f"docker could not be executed: {exc}")

    if proc.returncode != 0:
        detail = (proc.stderr or proc.stdout or "").strip().splitlines()
        first = detail[0] if detail else "no output"
        return DockerStatus(False, f"docker exited {proc.returncode}: {first}")

    version = (proc.stdout or "").strip()
    if not version:
        return DockerStatus(False, "docker reported no server version; the daemon is not ready")
    return DockerStatus(True, f"docker server {version}")


def skip_reason() -> str | None:
    """None when URSim work can proceed, else the reason to skip with.

    Kept separate from docker_status so tests read one string and the
    decision to skip is never made by inspecting an exception.
    """
    status = docker_status()
    return None if status.available else f"URSim unavailable: {status.reason}"
