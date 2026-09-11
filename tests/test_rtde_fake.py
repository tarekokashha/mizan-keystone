"""Tests for keystone.rtde_fake: the deterministic fakes of the ur_rtde interfaces.

These are not assert-on-calls mock tests only. The most important test in
this file is test_fidelity_against_real_ur_rtde: it compares the fake's
method surface against the installed ur_rtde 1.6.5, because a fake that
drifts from the real interface is worse than no fake at all.
"""
from __future__ import annotations

import inspect

import dashboard_client
import numpy as np
import pytest
import rtde_control
import rtde_receive

from keystone.rtde_fake import FakeControl, FakeDashboard, FakeReceive, FakeRTDE

CONTROL_METHODS = ["servoJ", "servoStop", "stopScript", "initPeriod", "waitPeriod",
                    "disconnect", "isConnected"]
RECEIVE_METHODS = ["getActualQ", "getActualQd", "getActualTCPPose", "getActualTCPForce",
                    "getTimestamp", "disconnect"]
DASHBOARD_METHODS = ["connect", "powerOn", "brakeRelease", "isInRemoteControl", "disconnect"]


# ---- every method the driver will call exists ------------------------------ #

@pytest.mark.parametrize("name", CONTROL_METHODS)
def test_fake_control_has_every_needed_method(name):
    assert callable(getattr(FakeControl, name, None)), f"FakeControl.{name} is missing"


@pytest.mark.parametrize("name", RECEIVE_METHODS)
def test_fake_receive_has_every_needed_method(name):
    assert callable(getattr(FakeReceive, name, None)), f"FakeReceive.{name} is missing"


@pytest.mark.parametrize("name", DASHBOARD_METHODS)
def test_fake_dashboard_has_every_needed_method(name):
    assert callable(getattr(FakeDashboard, name, None)), f"FakeDashboard.{name} is missing"


# ---- servoJ is recorded with its exact arguments ---------------------------- #

def test_servoj_is_recorded_with_its_exact_arguments():
    control = FakeControl()
    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    control.servoJ(q, 0.0, 0.0, 0.008, 0.1, 300.0)

    assert len(control.calls) == 1
    call = control.calls[0]
    assert call.method == "servoJ"
    assert call.args == (q, 0.0, 0.0, 0.008, 0.1, 300.0)
    assert call.kwargs == {}


def test_servoj_recording_is_immune_to_the_caller_mutating_its_list_afterwards():
    # Without a defensive copy, mutating `q` after the call would silently
    # rewrite what the fake claims was sent, which is exactly the kind of
    # mutable-state defeat this project keeps finding and fixing elsewhere.
    control = FakeControl()
    q = [0.0] * 6
    control.servoJ(q, 0.0, 0.0, 0.008, 0.1, 300.0)
    q[0] = 999.0
    assert control.calls[0].args[0] == [0.0] * 6


def test_servostop_and_stopscript_are_recorded():
    control = FakeControl()
    control.servoStop(5.0)
    control.stopScript()
    methods_called = [c.method for c in control.calls]
    assert "servoStop" in methods_called
    assert "stopScript" in methods_called
    servo_stop_call = next(c for c in control.calls if c.method == "servoStop")
    assert servo_stop_call.args == (5.0,)


def test_calls_since_returns_only_calls_after_the_mark():
    control = FakeControl()
    control.isConnected()
    mark = len(control.calls)
    control.servoStop()
    control.stopScript()
    since = control.calls_since(mark)
    assert [c.method for c in since] == ["servoStop", "stopScript"]


# ---- the control loop actually closes --------------------------------------- #

def test_getactualq_advances_toward_the_last_servoj_command():
    fake = FakeRTDE(control_hz=125.0, q0=np.zeros(6))
    target = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    fake.control.servoJ(target.tolist(), 0.0, 0.0, 0.008, 0.1, 300.0)

    q_before = np.array(fake.receive.getActualQ())
    q_after_many = q_before
    for _ in range(200):
        q_after_many = np.array(fake.receive.getActualQ())

    # It must have moved toward the target, and gotten close to it: this is
    # the property that makes a control loop meaningful against this fake,
    # not just a value that happens to be returned.
    assert q_after_many[0] > q_before[0]
    assert abs(q_after_many[0] - target[0]) < 1e-6


def test_getactualq_holds_position_when_nothing_has_been_commanded():
    fake = FakeRTDE(control_hz=125.0, q0=np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]))
    q1 = np.array(fake.receive.getActualQ())
    q2 = np.array(fake.receive.getActualQ())
    assert np.array_equal(q1, q2)
    assert np.array_equal(q1, np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]))


def test_servoj_on_control_is_visible_as_motion_on_receive():
    # The point of FakeRTDE: control and receive share one clock, so a
    # command issued on one is visible as motion on the other, without
    # threading them together by hand in every test.
    fake = FakeRTDE(control_hz=125.0)
    fake.control.servoJ([0.0, 0.0, 0.0, 0.0, 0.0, 0.5], 0.0, 0.0, 0.008, 0.1, 300.0)
    moved = False
    for _ in range(50):
        q = fake.receive.getActualQ()
        if q[5] > 0.0:
            moved = True
            break
    assert moved, "a servoJ command through .control produced no motion on .receive"


def test_getactualqd_reflects_the_last_step():
    fake = FakeRTDE(control_hz=125.0)
    fake.control.servoJ([1.0, 0, 0, 0, 0, 0], 0.0, 0.0, 0.008, 0.1, 300.0)
    fake.receive.getActualQ()
    qd = fake.receive.getActualQd()
    assert qd[0] > 0.0


# ---- getTimestamp advances by 1/control_hz per sample ------------------------ #

def test_gettimestamp_advances_by_one_over_control_hz_per_sample():
    fake = FakeRTDE(control_hz=125.0)
    t0 = fake.receive.getTimestamp()
    t1 = fake.receive.getTimestamp()
    t2 = fake.receive.getTimestamp()
    assert t1 - t0 == pytest.approx(1.0 / 125.0)
    assert t2 - t1 == pytest.approx(1.0 / 125.0)


def test_gettimestamp_uses_the_configured_control_hz():
    fake = FakeRTDE(control_hz=500.0)
    t0 = fake.receive.getTimestamp()
    t1 = fake.receive.getTimestamp()
    assert t1 - t0 == pytest.approx(1.0 / 500.0)


# ---- dashboard ---------------------------------------------------------------- #

def test_dashboard_methods_are_recorded_in_order():
    dash = FakeDashboard()
    dash.connect()
    dash.powerOn()
    dash.brakeRelease()
    dash.isInRemoteControl()
    dash.disconnect()
    assert [c.method for c in dash.calls] == [
        "connect", "powerOn", "brakeRelease", "isInRemoteControl", "disconnect"]


def test_dashboard_connect_records_its_timeout_argument():
    # The fake normalises to positional when it records (see
    # FakeDashboard.connect), so the value shows up in .args regardless of
    # whether the caller used keyword or positional syntax; the value
    # recorded is what matters, not the calling convention used to supply
    # it.
    dash = FakeDashboard()
    dash.connect(timeout_ms=5000)
    assert dash.calls[0].args == (5000,)


# ---- the fidelity check: this is the most important test in this file -------- #

def test_inspect_signature_fails_on_the_installed_pybind11_methods():
    # Documents and pins down a measured fact this whole fidelity check
    # depends on: ur_rtde 1.6.5 is a pybind11 extension, and
    # inspect.signature cannot introspect its builtin methods. If a future
    # ur_rtde version changes this, this test starts failing here, which is
    # the signal to upgrade test_fidelity_against_real_ur_rtde to compare
    # signatures directly instead of falling back to exists-and-callable.
    with pytest.raises(ValueError):
        inspect.signature(rtde_control.RTDEControlInterface.servoJ)


def test_fidelity_against_real_ur_rtde():
    """For every one of the 18 methods the driver needs, the fake's surface
    must not drift from what is actually installed.

    inspect.signature raises ValueError on ur_rtde's pybind11 methods
    (confirmed above), so the primary check here is the documented
    fallback: assert the method exists and is callable on both the real
    class and the fake. That fallback is itself asserted, not silently
    taken, via the assertion at the end of this test.

    As a stronger, best-effort supplement, ur_rtde's pybind11 docstrings
    carry a parsable first-line signature (e.g. "servoJ(self: ..., arg0:
    list[float], ..., arg5: float) -> bool"). This test extracts the real
    positional argument count from that line and compares it against the
    fake's own inspect.signature (which works fine, because the fake is
    plain Python), so an argument-count drift between fake and real is
    still caught even though inspect.signature cannot see the real side
    directly.
    """
    checks = [
        (rtde_control.RTDEControlInterface, FakeControl, CONTROL_METHODS),
        (rtde_receive.RTDEReceiveInterface, FakeReceive, RECEIVE_METHODS),
        (dashboard_client.DashboardClient, FakeDashboard, DASHBOARD_METHODS),
    ]

    signature_introspectable_on_real_class = []

    for real_cls, fake_cls, methods in checks:
        for name in methods:
            real_attr = getattr(real_cls, name, None)
            fake_attr = getattr(fake_cls, name, None)

            assert real_attr is not None, f"{real_cls.__name__}.{name} does not exist on the installed ur_rtde"
            assert callable(real_attr), f"{real_cls.__name__}.{name} exists but is not callable"
            assert fake_attr is not None, f"{fake_cls.__name__}.{name} is missing from the fake"
            assert callable(fake_attr), f"{fake_cls.__name__}.{name} exists but is not callable"

            try:
                inspect.signature(real_attr)
                signature_introspectable_on_real_class.append(f"{real_cls.__name__}.{name}")
                continue  # a real signature is available; nothing more to fall back to
            except (ValueError, TypeError):
                pass

            # Fallback confirmed necessary for this method. Cross-check
            # argument count against the docstring's pybind11 signature.
            doc = real_attr.__doc__ or ""
            first_line = doc.strip().splitlines()[0] if doc.strip() else ""
            assert first_line.startswith(f"{name}("), (
                f"{real_cls.__name__}.{name}'s docstring does not start with a "
                f"pybind11-style signature; cannot cross-check argument count: {doc!r}")
            # Count top-level commas between the parens on the first line,
            # then subtract one for `self`.
            paren_depth = 0
            arg_count = 0
            inside = False
            current_has_content = False
            for ch in first_line:
                if ch == "(":
                    if paren_depth == 0:
                        inside = True
                    else:
                        current_has_content = True
                    paren_depth += 1
                elif ch == ")":
                    paren_depth -= 1
                    if paren_depth == 0:
                        if current_has_content or arg_count > 0:
                            arg_count += 1
                        inside = False
                elif ch == "," and inside and paren_depth == 1:
                    arg_count += 1
                    current_has_content = False
                elif inside and paren_depth >= 1 and not ch.isspace():
                    current_has_content = True
            real_positional_count = arg_count - 1  # exclude self

            fake_params = [
                p for p in inspect.signature(fake_attr).parameters.values()
                if p.name != "self"
            ]
            assert len(fake_params) == real_positional_count, (
                f"{fake_cls.__name__}.{name} takes {len(fake_params)} argument(s) "
                f"but the installed ur_rtde's {real_cls.__name__}.{name} takes "
                f"{real_positional_count}, per its docstring signature {first_line!r}"
            )

    assert not signature_introspectable_on_real_class, (
        "inspect.signature succeeded on installed ur_rtde pybind11 method(s) "
        f"{signature_introspectable_on_real_class}; this contradicts the measured "
        "premise of this fidelity check (see "
        "test_inspect_signature_fails_on_the_installed_pybind11_methods) and this "
        "test should be upgraded to compare signatures directly for those methods."
    )
