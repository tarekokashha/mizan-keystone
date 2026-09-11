"""keystone.timing: measure the control loop's cost, and assert nothing.

This module contains no threshold, no pass/fail rule and no comparison
against a target rate. That is deliberate and it is pre-registered in
PROTOCOL.md section 6. The vendor kit's `ursim_smoke.py` declares a CI gate
of "rate >= 480 Hz, p99.9 jitter < 2500 us on a laptop", a number written
without running it on a platform that was not specified. The kit's
operating contract forbids writing a robot number that was not measured,
and a latency gate is a robot number. A gate may be set later, from an
observed distribution, with the platform recorded beside it. Not here.

What this measures, and what it does not, matters more than the numbers.

MEASURED: the wall-clock cost of one full cycle of the composed software
stack, policy action in to servoJ out, with RTDE replaced by the
deterministic in-process fake. That answers a real question: whether the
SENTINEL kernel and Shield are fast enough to sit inside a control loop at
all, and what they cost per cycle. Running the driver alone and the
composed stack separately makes the safety layer's own cost visible as a
difference rather than an assertion.

NOT MEASURED: end-to-end RTDE timing against a controller. That crosses a
socket, a NAT and a scheduler none of which this touches. No number
produced here describes it, and none should be quoted as if it did. See
PROTOCOL.md section 2.

All timing uses time.perf_counter(). On Windows time.monotonic() is
GetTickCount64() with a resolution of 15.625 ms, while a 500 Hz control
period is 2 ms, so every sub-period figure taken with it would be a
multiple of 15.625 ms or zero. perf_counter() is QueryPerformanceCounter()
at 100 ns. M-01 SENTINEL found the two clocks 5.558460 s apart on this
machine, which is also why the kernel never compares them.
"""
from __future__ import annotations

import json
import platform
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from keystone.config import UR5eConfig
from keystone.follower import UR5eFollower
from keystone.guarded import guarded_ur5e
from keystone.rtde_fake import FakeRTDE

# Budget, fixed in PROTOCOL.md section 5 before any measurement was taken.
CYCLES = 20000
WARMUP = 1000
REPEATS = 3

N_JOINTS = 6
# A posture well inside the declared envelope. The loop below perturbs it by
# a small amount so the kernel does real work on every cycle rather than
# short-circuiting on an unchanged target. These are simulation inputs, not
# measurements of any robot.
BASE_Q = np.array([0.0, -1.2, 1.2, -1.5, -1.5708, 0.0])
WOBBLE_RAD = 0.001


@dataclass(frozen=True)
class TimingResult:
    """One measurement run. Percentiles, not a mean: the tail is the part
    that breaks a control loop, and a mean hides it.
    """

    label: str
    cycles: int
    warmup_discarded: int
    rate_hz: float
    p50_us: float
    p99_us: float
    p999_us: float
    max_us: float
    min_us: float
    total_s: float

    def table_row(self) -> str:
        return (f"{self.label:<22} {self.rate_hz:>12.1f} {self.p50_us:>10.2f} "
                f"{self.p99_us:>10.2f} {self.p999_us:>10.2f} {self.max_us:>10.2f}")


def platform_record() -> dict[str, Any]:
    """The platform a timing number was taken on. A timing number without
    one is not a measurement, so this is recorded beside every run.
    """
    return {
        "python": sys.version.split()[0],
        "machine": platform.machine(),
        "system": f"{platform.system()} {platform.version()}",
        "perf_counter_resolution_s": time.get_clock_info("perf_counter").resolution,
        "perf_counter_impl": time.get_clock_info("perf_counter").implementation,
        "realtime_scheduling": False,
    }


def measure(step: Callable[[int], None], label: str,
            cycles: int = CYCLES, warmup: int = WARMUP) -> TimingResult:
    """Time `step` once per cycle and summarise the distribution.

    The warmup cycles are run and discarded, which is declared in
    PROTOCOL.md section 5 rather than chosen after seeing the data. Samples
    are collected into a preallocated array so the measurement does not
    time its own list growth.
    """
    for i in range(warmup):
        step(i)

    samples = np.empty(cycles, dtype=np.float64)
    t_start = time.perf_counter()
    for i in range(cycles):
        t0 = time.perf_counter()
        step(warmup + i)
        samples[i] = time.perf_counter() - t0
    total = time.perf_counter() - t_start

    us = samples * 1e6
    return TimingResult(
        label=label,
        cycles=cycles,
        warmup_discarded=warmup,
        rate_hz=cycles / total if total > 0 else float("inf"),
        p50_us=float(np.percentile(us, 50)),
        p99_us=float(np.percentile(us, 99)),
        p999_us=float(np.percentile(us, 99.9)),
        max_us=float(us.max()),
        min_us=float(us.min()),
        total_s=total,
    )


def _target(i: int) -> np.ndarray:
    """A small, in-envelope perturbation so every cycle does real work."""
    q = BASE_Q.copy()
    q[0] += WOBBLE_RAD * np.sin(i * 0.01)
    return q


def driver_only_step() -> Callable[[int], None]:
    """One cycle of the bare driver: observe, then command. No kernel."""
    cfg = UR5eConfig.declared()
    robot = UR5eFollower(cfg, rtde=FakeRTDE(control_hz=cfg.control_hz, q0=BASE_Q))
    robot.connect()

    def step(i: int) -> None:
        robot.get_observation()
        robot.send_action({"joint_position": _target(i)})

    return step


def composed_step() -> Callable[[int], None]:
    """One cycle of the composed stack: observe, then command through the
    Shield and the kernel. The difference against driver_only is what the
    safety layer costs.
    """
    cfg = UR5eConfig.declared()
    robot = guarded_ur5e(cfg, rtde=FakeRTDE(control_hz=cfg.control_hz, q0=BASE_Q))
    robot.connect()

    def step(i: int) -> None:
        robot.get_observation()
        robot.send_action({"joint_position": _target(i)})

    return step


def run_all(cycles: int = CYCLES, warmup: int = WARMUP,
            repeats: int = REPEATS) -> dict[str, Any]:
    """Every repeat is reported, not the best of them (PROTOCOL.md 5)."""
    runs: list[TimingResult] = []
    for r in range(repeats):
        runs.append(measure(driver_only_step(), f"driver_only r{r + 1}", cycles, warmup))
        runs.append(measure(composed_step(), f"composed r{r + 1}", cycles, warmup))
    return {"platform": platform_record(), "runs": [asdict(x) for x in runs],
            "budget": {"cycles": cycles, "warmup": warmup, "repeats": repeats}}


def main() -> int:
    out = run_all()
    print(f"{'run':<22} {'rate Hz':>12} {'p50 us':>10} {'p99 us':>10} "
          f"{'p99.9 us':>10} {'max us':>10}")
    for row in out["runs"]:
        print(TimingResult(**row).table_row())
    print()
    print("These are software costs measured against the deterministic fake.")
    print("They are NOT end-to-end RTDE timing; see PROTOCOL.md section 2.")

    results = Path("results")
    results.mkdir(exist_ok=True)
    path = results / "timing.json"
    path.write_text(json.dumps(out, indent=2), encoding="utf-8")
    print(f"\nwritten: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
