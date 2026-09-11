"""Tests for keystone.timing.

The most important test here asserts an absence: that the module contains no
threshold. PROTOCOL.md section 6 commits this programme to measuring timing
and reporting it rather than gating on a number nobody measured, and an
absence that nothing enforces is an absence that will not survive the next
person in a hurry.
"""
from __future__ import annotations

import inspect
import re

import numpy as np
import pytest

import keystone.timing as timing_module
from keystone.timing import TimingResult, measure, platform_record


def test_the_module_contains_no_threshold_and_no_pass_fail_rule():
    """PROTOCOL.md section 6, enforced rather than trusted.

    The vendor kit's ursim_smoke.py declares "rate >= 480 Hz, p99.9 jitter
    < 2500 us on a laptop", written without running it on a platform that
    was not specified. This programme does not adopt a threshold it has not
    measured, and a latency gate is a robot number. If a future change adds
    one here, this test fails and whoever added it has to argue for it in
    PROTOCOL.md first.
    """
    src = inspect.getsource(timing_module)
    # Strip comments and docstrings: the prose explains the thresholds this
    # module refuses to adopt, and quoting them is not adopting them.
    code = "\n".join(l.split("#")[0] for l in src.splitlines())
    code = re.sub(r'""".*?"""', "", code, flags=re.S)

    banned = re.findall(
        r"(?:rate_hz|p50_us|p99_us|p999_us|max_us|min_us)\s*[<>]=?|"
        r"assert\s|"
        r"\bpytest\.fail\b|\braise\s+AssertionError",
        code)
    assert banned == [], f"keystone/timing.py has grown a pass/fail rule: {banned}"

    # And the result type must not carry a verdict field either.
    assert not any(f in TimingResult.__dataclass_fields__
                   for f in ("passed", "ok", "within_budget", "verdict"))


def test_measure_reports_the_true_percentiles_of_a_known_distribution(monkeypatch):
    """Percentile arithmetic, checked against values known in advance.

    Wall-clock timing cannot test this: the numbers are never known. So the
    clock is replaced with one that advances by a chosen amount per cycle,
    making every sample exact.
    """
    # 100 cycles whose durations are 1..100 microseconds, plus a warmup of 5
    # that must not appear in the statistics.
    durations_us = list(range(1, 101))
    warmup = 5
    # measure() reads no clock during warmup. It reads once for t_start,
    # twice per timed cycle, and once more for the total. Anything else
    # misaligns the sequence and the percentiles come out as zeros, which is
    # how this test was first written and how it caught its own error.
    ticks: list[float] = []
    t = 0.0
    ticks.append(t)
    for d in durations_us:
        ticks.append(t)
        t += d * 1e-6
        ticks.append(t)
    ticks.append(t)

    it = iter(ticks)
    monkeypatch.setattr(timing_module.time, "perf_counter", lambda: next(it))

    r = measure(lambda i: None, "known", cycles=len(durations_us), warmup=warmup)

    expected = np.array(durations_us, dtype=float)
    assert r.p50_us == pytest.approx(float(np.percentile(expected, 50)))
    assert r.p99_us == pytest.approx(float(np.percentile(expected, 99)))
    assert r.p999_us == pytest.approx(float(np.percentile(expected, 99.9)))
    assert r.max_us == pytest.approx(100.0)
    assert r.min_us == pytest.approx(1.0)
    assert r.cycles == 100
    assert r.warmup_discarded == warmup


def test_warmup_cycles_are_run_and_then_excluded_from_the_statistics():
    """A warmup that is not run is a different experiment from one that is
    run and discarded, and only the second is what PROTOCOL.md declares.
    """
    seen: list[int] = []
    r = measure(seen.append, "warmup", cycles=10, warmup=4)

    assert len(seen) == 14, "warmup cycles were not actually executed"
    assert seen == list(range(14)), "step did not receive a continuous index"
    assert r.cycles == 10, "warmup cycles leaked into the reported sample count"


def test_timing_uses_perf_counter_and_not_the_coarse_windows_clock():
    """On Windows time.monotonic() resolves to 15.625 ms against a 2 ms
    control period, so a jitter figure taken with it would be quantisation
    artefact. This asserts the module actually reads perf_counter.
    """
    calls: list[str] = []
    real = timing_module.time.perf_counter

    class _Watch:
        def perf_counter(self) -> float:
            calls.append("perf_counter")
            return real()

        def monotonic(self) -> float:
            calls.append("monotonic")
            return real()

        def get_clock_info(self, name):
            return timing_module.time.get_clock_info(name)

    saved = timing_module.time
    timing_module.time = _Watch()
    try:
        measure(lambda i: None, "clock", cycles=5, warmup=1)
    finally:
        timing_module.time = saved

    assert "perf_counter" in calls
    assert "monotonic" not in calls, "timing read the 15.625 ms Windows clock"


def test_the_platform_is_recorded_because_a_timing_number_without_one_is_not_one():
    p = platform_record()
    for key in ("python", "machine", "system", "perf_counter_resolution_s",
                "perf_counter_impl", "realtime_scheduling"):
        assert key in p, f"platform record is missing {key}"
    assert p["realtime_scheduling"] is False
    assert p["perf_counter_resolution_s"] > 0
