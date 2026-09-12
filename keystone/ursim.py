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


# ---- container lifecycle ---------------------------------------------------- #
#
# Everything below runs only where the Docker engine answers. On the
# development host it never did, so this path was written against GitHub's
# ubuntu runners, where `docker server 28.0.4` was observed. Timeouts are
# generous because the image is large and the controller takes time to boot,
# but every one is bounded, for the reason in the module docstring.

PULL_TIMEOUT_S = 900.0
BOOT_TIMEOUT_S = 240.0
STOP_TIMEOUT_S = 60.0
CONTAINER_NAME = "keystone-ursim"


@dataclass(frozen=True)
class ContainerHandle:
    """A running URSim container, and how it was reached."""

    name: str
    container_id: str
    image: str


def _docker(args: Sequence[str], timeout_s: float,
            runner: Callable[..., subprocess.CompletedProcess] = _run):
    return runner(["docker", *args], timeout_s)


def pull_image(image: str = URSIM_IMAGE, timeout_s: float = PULL_TIMEOUT_S) -> DockerStatus:
    """Pull the URSim image, reporting rather than raising."""
    try:
        proc = _docker(["pull", image], timeout_s)
    except subprocess.TimeoutExpired:
        return DockerStatus(False, f"docker pull {image} exceeded {timeout_s:g}s")
    except OSError as exc:
        return DockerStatus(False, f"docker pull could not run: {exc}")
    if proc.returncode != 0:
        first = (proc.stderr or proc.stdout or "no output").strip().splitlines()[0]
        return DockerStatus(False, f"docker pull {image} failed: {first}")
    return DockerStatus(True, f"pulled {image}")


def wait_for_tcp(host: str, port: int, timeout_s: float) -> bool:
    """True once `port` accepts a connection, False if `timeout_s` elapses.

    URSim reports nothing useful on stdout while the controller boots, so
    the port accepting a connection is the only honest readiness signal
    available from outside the container.
    """
    import socket
    import time as _time

    deadline = _time.monotonic() + timeout_s
    while _time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=2.0):
                return True
        except OSError:
            _time.sleep(1.0)
    return False


def start_container(image: str = URSIM_IMAGE, name: str = CONTAINER_NAME,
                    timeout_s: float = 120.0) -> tuple[ContainerHandle | None, str]:
    """Start URSim detached, publishing the dashboard and RTDE ports."""
    _docker(["rm", "-f", name], STOP_TIMEOUT_S)  # ignore result; may not exist
    args = [
        "run", "-d", "--name", name,
        "-p", f"{URSIM_DASHBOARD_PORT}:{URSIM_DASHBOARD_PORT}",
        "-p", f"{URSIM_RTDE_PORT}:{URSIM_RTDE_PORT}",
        image,
    ]
    try:
        proc = _docker(args, timeout_s)
    except subprocess.TimeoutExpired:
        return None, f"docker run exceeded {timeout_s:g}s"
    except OSError as exc:
        return None, f"docker run could not execute: {exc}"
    if proc.returncode != 0:
        first = (proc.stderr or proc.stdout or "no output").strip().splitlines()[0]
        return None, f"docker run failed: {first}"
    return ContainerHandle(name=name, container_id=proc.stdout.strip(), image=image), "started"


def stop_container(name: str = CONTAINER_NAME) -> None:
    """Remove the container. Best effort: a leaked container on a throwaway
    CI runner is not worth raising over, and raising here would mask the
    test failure that sent us into teardown.
    """
    try:
        _docker(["rm", "-f", name], STOP_TIMEOUT_S)
    except (subprocess.TimeoutExpired, OSError):
        pass


def container_logs(name: str = CONTAINER_NAME, tail: int = 40) -> str:
    try:
        proc = _docker(["logs", "--tail", str(tail), name], 30.0)
    except (subprocess.TimeoutExpired, OSError) as exc:
        return f"(could not read logs: {exc})"
    return ((proc.stdout or "") + (proc.stderr or "")).strip() or "(no output)"


def wait_for_rtde(host: str, timeout_s: float = BOOT_TIMEOUT_S) -> tuple[bool, str]:
    """Wait until RTDE will actually open a session, not merely accept a socket.

    Measured on a GitHub ubuntu runner: URSim's RTDE port starts listening
    well before the controller can serve a session, so wait_for_tcp returns
    True and RTDEReceiveInterface then fails with

        RuntimeError: read: Connection reset by peer

    A port accepting a connection is not a controller that is ready. The only
    honest readiness signal for "RTDE works" is RTDE working, so this
    constructs the real interface and retries until it succeeds or the
    deadline passes, and reports the last error rather than swallowing it.
    """
    import time as _time

    try:
        import rtde_receive
    except ImportError as exc:  # pragma: no cover - ur_rtde is a hard dependency
        return False, f"ur_rtde is not importable: {exc}"

    deadline = _time.monotonic() + timeout_s
    last = "no attempt was made"
    attempts = 0
    while _time.monotonic() < deadline:
        attempts += 1
        try:
            rtde = rtde_receive.RTDEReceiveInterface(host)
        except Exception as exc:
            last = f"{type(exc).__name__}: {exc}"
            _time.sleep(3.0)
            continue
        try:
            if rtde.isConnected():
                return True, f"RTDE session established after {attempts} attempts"
            last = "constructed but isConnected() was False"
        finally:
            try:
                rtde.disconnect()
            except Exception:
                pass
        _time.sleep(3.0)
    return False, f"no RTDE session within {timeout_s:g}s after {attempts} attempts; last error: {last}"
