"""Tests for keystone.rtde_fake: the deterministic fakes of the ur_rtde interfaces.

These are not assert-on-calls mock tests only. The most important test in
this file is test_fidelity_against_real_ur_rtde: it compares the fake's
method surface against the installed ur_rtde 1.6.5, because a fake that
drifts from the real interface is worse than no fake at all.
"""
from __future__ import annotations

import datetime
import inspect

import dashboard_client
import numpy as np
import pytest
import rtde_control
import rtde_receive

from keystone.rtde_fake import (N_JOINTS, FakeControl, FakeDashboard,
                                FakeReceive, FakeRTDE)

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


# ---- Finding 5: last_servoj_q, the property claim (A) is asserted through ---- #
#
# tests/test_guarded.py asserts claim (A) for all fifteen attacks through
# `np.array_equal(fake.control.last_servoj_q, robot.last_verdict.action.q)`.
# Until these tests existed this file referenced that property nowhere, so the
# property the central claim is read from was covered only by the tests that
# already trust it. Measured: a last_servoj_q returning a fixed vector, and one
# returning the internal target instead of a copy, both left the whole suite
# green.

def test_last_servoj_q_is_none_before_any_servoj():
    assert FakeControl().last_servoj_q is None


def test_last_servoj_q_is_the_joint_vector_most_recently_commanded():
    control = FakeControl()
    q = [0.11, -0.22, 0.33, -0.44, 0.55, -0.66]
    control.servoJ(q, 0.0, 0.0, 0.008, 0.1, 300.0)
    assert np.array_equal(control.last_servoj_q, np.array(q))


def test_last_servoj_q_tracks_the_latest_command_not_the_first():
    control = FakeControl()
    control.servoJ([0.1] * N_JOINTS, 0.0, 0.0, 0.008, 0.1, 300.0)
    control.servoJ([0.2] * N_JOINTS, 0.0, 0.0, 0.008, 0.1, 300.0)
    assert np.array_equal(control.last_servoj_q, np.full(N_JOINTS, 0.2))


def test_last_servoj_q_is_a_copy_not_the_internal_target():
    """The copy is what stops a consumer rewriting the recorded target
    through the value it was handed.

    If the property returned _clock.q_target itself, anything holding that
    value could edit the very evidence claim (A) is read from, and the
    simulation would follow the edit.
    """
    fake = FakeRTDE(control_hz=125.0)
    fake.control.servoJ([0.5] * N_JOINTS, 0.0, 0.0, 0.008, 0.1, 300.0)

    handed_out = fake.control.last_servoj_q
    handed_out[0] = 999.0

    # Re-reading still reports the commanded target ...
    assert fake.control.last_servoj_q[0] == 0.5
    # ... and the simulation is still converging on it, not on 999.
    for _ in range(200):
        q = fake.receive.getActualQ()
    assert q[0] == pytest.approx(0.5)


# ---- Finding 9: _freeze's ndarray branch, which is the branch that matters -- #

def test_servoj_recording_is_immune_to_the_caller_mutating_its_ndarray_afterwards():
    """The list branch of _freeze already had a test; the ndarray branch did
    not, and deleting it left the suite green.

    sentinel's kernel emits float64 arrays, so the ndarray branch is the one
    the recorded history behind claim (A) is actually made of. The existing
    immunity test passes a Python list and never reaches it.
    """
    control = FakeControl()
    q = np.zeros(N_JOINTS)
    control.servoJ(q, 0.0, 0.0, 0.008, 0.1, 300.0)
    q[0] = 999.0
    assert np.array_equal(control.calls[0].args[0], np.zeros(N_JOINTS))


# ---- Finding 10: the recorded history is immutable in BOTH directions ------- #
#
# _freeze guarded one direction only: the caller mutating the list it passed
# in. The other direction was open. Measured before this was closed:
# `fake.control.calls[0].args[0][0] = 999.0` rewrote the record to
# [999.0, 0.1, 0.1, 0.1, 0.1, 0.1]. Call.args is a tuple, but the objects
# inside it held the only copy. Since .calls is the evidence base for claim
# (A), a history its reader can edit proves nothing about what was sent.

def test_a_recorded_list_argument_cannot_be_rewritten_through_calls():
    control = FakeControl()
    original = [0.0, 0.1, 0.1, 0.1, 0.1, 0.1]
    control.servoJ(list(original), 0.0, 0.0, 0.008, 0.1, 300.0)

    recorded = control.calls[0].args[0]
    with pytest.raises(TypeError):
        recorded[0] = 999.0
    with pytest.raises(TypeError):
        del recorded[0]
    for label, mutate in [
        ("append", lambda r: r.append(1.0)),
        ("clear", lambda r: r.clear()),
        ("extend", lambda r: r.extend([1.0])),
        ("insert", lambda r: r.insert(0, 1.0)),
        ("pop", lambda r: r.pop()),
        ("remove", lambda r: r.remove(0.1)),
        ("reverse", lambda r: r.reverse()),
        ("sort", lambda r: r.sort()),
    ]:
        with pytest.raises(TypeError):
            mutate(control.calls[0].args[0])

    assert control.calls[0].args[0] == original
    # Value equality against a plain list must survive the freeze: the
    # readers of .calls in tests/test_follower.py compare against one.
    assert control.calls[0].args[0] == [0.0, 0.1, 0.1, 0.1, 0.1, 0.1]


def test_a_recorded_ndarray_argument_cannot_be_rewritten_through_calls():
    # The type sentinel's kernel actually emits, so this is the case that
    # decides whether claim (A)'s evidence base is writable.
    control = FakeControl()
    control.servoJ(np.zeros(N_JOINTS), 0.0, 0.0, 0.008, 0.1, 300.0)

    recorded = control.calls[0].args[0]
    assert not recorded.flags.writeable
    with pytest.raises(ValueError):
        recorded[0] = 999.0
    assert np.array_equal(control.calls[0].args[0], np.zeros(N_JOINTS))


def test_the_call_history_itself_cannot_be_appended_to_or_replaced():
    """A record that can be rewritten is one defeat; a history that can gain
    a call that never happened, or lose one that did, is the same defeat one
    level up."""
    control = FakeControl()
    control.isConnected()

    assert isinstance(control.calls, tuple)
    with pytest.raises(AttributeError):
        control.calls.append(control.calls[0])
    with pytest.raises(AttributeError):
        control.calls = []

    assert [c.method for c in control.calls] == ["isConnected"]


def test_recorded_kwargs_cannot_be_rewritten_through_calls():
    """The recorder's keyword path, which no fake method currently uses.

    Every method here records positionally, so this exercises _record
    directly. The path exists and freezes its values, and an untested freeze
    is the kind that quietly stops freezing.
    """
    control = FakeControl()
    control._record("probe", alpha=[1.0, 2.0])

    recorded = control.calls[0].kwargs
    assert recorded == {"alpha": [1.0, 2.0]}
    with pytest.raises(TypeError):
        recorded["injected"] = "a keyword that was never passed"
    with pytest.raises(TypeError):
        recorded["alpha"][0] = 999.0
    assert control.calls[0].kwargs == {"alpha": [1.0, 2.0]}


# ---- Finding 11: fidelity beyond argument counts ---------------------------- #
#
# test_fidelity_against_real_ur_rtde compares argument counts only. It says
# nothing about what comes back, and five return-value defects survived it:
# initPeriod returning float instead of datetime.timedelta, getActualQ
# returning an ndarray instead of list[float], servoStop no longer stopping,
# servoJ returning False, and isInRemoteControl returning False.
#
# Measured: ur_rtde 1.6.5's pybind11 docstrings declare a return type on the
# first line of all 18 methods the driver needs (e.g. "getActualQ(self: ...)
# -> list[float]"). The real interfaces cannot be constructed offline, since
# their constructors connect to hardware, but the declared type is readable
# without constructing anything. So the return-type half of the contract is
# checkable here; the behaviour of the real C++ validators is not, and this
# file asserts nothing about it.

RETURN_FIDELITY_CASES = [
    (rtde_control.RTDEControlInterface, "control", "servoJ",
     ([0.0] * N_JOINTS, 0.0, 0.0, 0.008, 0.1, 300.0)),
    (rtde_control.RTDEControlInterface, "control", "servoStop", ()),
    (rtde_control.RTDEControlInterface, "control", "stopScript", ()),
    (rtde_control.RTDEControlInterface, "control", "initPeriod", ()),
    (rtde_control.RTDEControlInterface, "control", "waitPeriod",
     (datetime.timedelta(seconds=0.0),)),
    (rtde_control.RTDEControlInterface, "control", "disconnect", ()),
    (rtde_control.RTDEControlInterface, "control", "isConnected", ()),
    (rtde_receive.RTDEReceiveInterface, "receive", "getActualQ", ()),
    (rtde_receive.RTDEReceiveInterface, "receive", "getActualQd", ()),
    (rtde_receive.RTDEReceiveInterface, "receive", "getActualTCPPose", ()),
    (rtde_receive.RTDEReceiveInterface, "receive", "getActualTCPForce", ()),
    (rtde_receive.RTDEReceiveInterface, "receive", "getTimestamp", ()),
    (rtde_receive.RTDEReceiveInterface, "receive", "disconnect", ()),
    (dashboard_client.DashboardClient, "dashboard", "connect", ()),
    (dashboard_client.DashboardClient, "dashboard", "powerOn", ()),
    (dashboard_client.DashboardClient, "dashboard", "brakeRelease", ()),
    (dashboard_client.DashboardClient, "dashboard", "isInRemoteControl", ()),
    (dashboard_client.DashboardClient, "dashboard", "disconnect", ()),
]


def test_the_return_type_table_covers_every_method_the_driver_needs():
    """Without this, dropping a case from RETURN_FIDELITY_CASES would silently
    stop checking that method, the same way an empty REGISTRY would make a
    parametrised test pass by collecting nothing."""
    covered = {(role, name) for _, role, name, _ in RETURN_FIDELITY_CASES}
    expected = ({("control", m) for m in CONTROL_METHODS}
                | {("receive", m) for m in RECEIVE_METHODS}
                | {("dashboard", m) for m in DASHBOARD_METHODS})
    assert covered == expected
    assert len(RETURN_FIDELITY_CASES) == 18


def _declared_return_type(real_attr, name):
    """The return type ur_rtde's own docstring declares for `name`."""
    doc = (real_attr.__doc__ or "").strip()
    first_line = doc.splitlines()[0] if doc else ""
    assert first_line.startswith(f"{name}("), (
        f"{name}'s docstring does not start with a pybind11-style signature; "
        f"the declared return type cannot be read: {doc!r}")
    assert " -> " in first_line, (
        f"{name}'s docstring signature declares no return type: {first_line!r}")
    return first_line.rsplit(" -> ", 1)[1].strip()


def _assert_matches_declared_type(declared, value, where):
    if declared == "None":
        assert value is None, f"{where} declares -> None but returned {value!r}"
    elif declared == "bool":
        assert type(value) is bool, (
            f"{where} declares -> bool but returned {type(value).__name__}")
    elif declared == "float":
        assert type(value) is float, (
            f"{where} declares -> float but returned {type(value).__name__}")
    elif declared == "datetime.timedelta":
        assert type(value) is datetime.timedelta, (
            f"{where} declares -> datetime.timedelta but returned "
            f"{type(value).__name__}")
    elif declared == "list[float]":
        assert type(value) is list, (
            f"{where} declares -> list[float] but returned "
            f"{type(value).__name__}")
        assert all(type(x) is float for x in value), (
            f"{where} declares -> list[float] but returned element types "
            f"{sorted({type(x).__name__ for x in value})}")
    else:
        pytest.fail(
            f"{where}: the installed ur_rtde declares the return type "
            f"{declared!r}, which this check does not know how to verify. Add "
            f"it here rather than letting the check pass vacuously.")


@pytest.mark.parametrize(
    "real_cls,role,name,args", RETURN_FIDELITY_CASES,
    ids=[f"{role}.{name}" for _, role, name, _ in RETURN_FIDELITY_CASES])
def test_return_type_fidelity_against_real_ur_rtde(real_cls, role, name, args):
    """What the fake returns must match the return type the installed
    ur_rtde declares for the same method."""
    declared = _declared_return_type(getattr(real_cls, name), name)
    fake_iface = getattr(FakeRTDE(control_hz=125.0), role)
    value = getattr(fake_iface, name)(*args)
    _assert_matches_declared_type(
        declared, value, f"{type(fake_iface).__name__}.{name}")


def test_initperiod_hands_waitperiod_the_type_the_real_interface_declares():
    """initPeriod/waitPeriod is a typed handoff, not a pair of opaque calls.

    Measured from the installed ur_rtde's docstrings below:
    `initPeriod(...) -> datetime.timedelta` and
    `waitPeriod(self, arg0: datetime.timedelta) -> None`. A float on either
    side of that handoff would let a type mismatch survive to hardware.
    """
    real_init = _first_doc_line(rtde_control.RTDEControlInterface.initPeriod)
    real_wait = _first_doc_line(rtde_control.RTDEControlInterface.waitPeriod)
    assert real_init.endswith("-> datetime.timedelta"), real_init
    assert "arg0: datetime.timedelta" in real_wait, real_wait

    control = FakeControl()
    t_start = control.initPeriod()
    assert type(t_start) is datetime.timedelta
    assert control.waitPeriod(t_start) is None
    assert control.calls[-1].args == (t_start,)


def _first_doc_line(attr):
    doc = (attr.__doc__ or "").strip()
    return doc.splitlines()[0] if doc else ""


def test_servoj_returns_true_the_real_interfaces_success_signal():
    """servoJ -> bool is the real interface's failure signal, so returning a
    bool is not enough: an accepted command must report success. A fake that
    always reported failure would put every caller that checks the return
    value on its error path, in every test, invisibly."""
    control = FakeControl()
    assert control.servoJ([0.0] * N_JOINTS, 0.0, 0.0, 0.008, 0.1, 300.0) is True


def test_servostop_returns_true():
    assert FakeControl().servoStop() is True


def test_isconnected_reports_the_connection_state_and_disconnect_changes_it():
    control = FakeControl()
    assert control.isConnected() is True
    control.disconnect()
    assert control.isConnected() is False


def test_isinremotecontrol_reports_that_the_robot_is_in_remote_control():
    """A fake reporting False here would silently send every readiness check
    in the suite down its not-ready path."""
    assert FakeDashboard().isInRemoteControl() is True


def test_servostop_actually_stops_the_simulated_motion():
    """servoStop's entire job is to stop, and its stopping behaviour had no
    test: deleting the line that collapses the target left the suite green."""
    fake = FakeRTDE(control_hz=125.0)
    fake.control.servoJ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.008, 0.1, 300.0)
    for _ in range(3):
        fake.receive.getActualQ()
    stopped_at = fake.receive.getActualQ()[0]
    assert stopped_at > 0.0, "precondition: the simulation was moving"

    fake.control.servoStop()

    after = [fake.receive.getActualQ()[0] for _ in range(10)]
    assert after == [stopped_at] * 10, (
        f"servoStop did not stop the simulated motion: {after}")


# ---- servoJ records the attempt, and accepts what it is given --------------- #

@pytest.mark.parametrize("bad_q", [[0.0] * 3, [0.0] * 7],
                         ids=["3-element", "7-element"])
def test_servoj_records_the_attempt_even_when_it_cannot_be_simulated(bad_q):
    """A deliberate choice, made explicit here rather than left as an accident.

    A q that is not N_JOINTS long cannot be simulated: the fake has nowhere
    to put the target, and numpy's reshape raises. The call is recorded
    anyway, because .calls answers "what did the driver put on the wire",
    and claim (A) is read from it. A history that dropped the calls the
    simulation could not model would let a driver put a command past the
    evidence base simply by malforming it. An evidence log may over-report;
    it must never under-report.

    The ValueError is numpy's, raised by reshape. It is simulation
    bookkeeping, not validation, and this fake performs no safety checks of
    any kind by design.
    """
    control = FakeControl()
    with pytest.raises(ValueError):
        control.servoJ(bad_q, 0.0, 0.0, 0.008, 0.1, 300.0)

    assert [c.method for c in control.calls] == ["servoJ"]
    assert control.calls[0].args[0] == bad_q
    # Nothing could be simulated, so no target was ever set.
    assert control.last_servoj_q is None


@pytest.mark.parametrize("bad", [np.nan, np.inf, -np.inf],
                         ids=["nan", "inf", "-inf"])
def test_servoj_accepts_non_finite_targets_because_the_fake_is_a_dumb_transport(bad):
    """Accepting NaN and inf here is required, not a gap to be closed.

    The architecture depends on this fake being a faithful dumb transport so
    that the driver's lack of safety logic stays observable. tests/
    test_guarded.py proves the Shield stops adversarial values, and that
    proof is only worth having if those values would otherwise have reached
    servoJ. A fake that rejected them would make the attack impossible to
    express and so prove the Shield unnecessary.

    Whether the real UR5e controller would reject such a target is NOT
    measurable offline: rtde_control.RTDEControlInterface connects to
    hardware in its constructor, so its C++ validators cannot be reached
    from this suite. This test pins the fake's behaviour and claims nothing
    about the robot's.
    """
    control = FakeControl()
    assert control.servoJ([bad] + [0.0] * 5, 0.0, 0.0, 0.008, 0.1, 300.0) is True
    assert len(control.calls) == 1

    commanded = control.last_servoj_q[0]
    assert np.isnan(commanded) if np.isnan(bad) else commanded == bad


def test_a_nan_target_poisons_the_state_and_servostop_does_not_clear_it():
    """Pinned deliberately, not fixed.

    One NaN command makes the simulated position non-finite for good: NaN
    propagates through the step arithmetic, and servoStop cannot recover it
    because servoStop halts AT the current position and that position is
    itself NaN. A later well-formed command does not recover it either.

    Sanitising the state would hide from the kernel exactly the non-finite
    condition its guards exist to catch, and tests/test_guarded.py drives
    the NaN path through this fake. What a real controller would do with a
    NaN target is not measurable offline, so the fake claims nothing about
    it; this test documents only what the fake does.
    """
    fake = FakeRTDE(control_hz=125.0)
    fake.control.servoJ([np.nan] * N_JOINTS, 0.0, 0.0, 0.008, 0.1, 300.0)
    assert all(np.isnan(x) for x in fake.receive.getActualQ())

    fake.control.servoStop()
    assert all(np.isnan(x) for x in fake.receive.getActualQ()), (
        "servoStop cleared the non-finite simulated state; that is not the "
        "behaviour this fake documents")

    fake.control.servoJ([0.0] * N_JOINTS, 0.0, 0.0, 0.008, 0.1, 300.0)
    assert all(np.isnan(x) for x in fake.receive.getActualQ()), (
        "a later well-formed command recovered the non-finite state; that is "
        "not the behaviour this fake documents")


# ---- the stalled RTDE stream ------------------------------------------------ #

def test_freeze_stream_stands_the_timestamp_and_the_joint_data_still():
    """A stalled stream serves the same sample again, so neither the
    controller's clock nor its joint data advances. This is the condition
    the kernel's staleness guard exists to catch."""
    fake = FakeRTDE(control_hz=125.0)
    fake.control.servoJ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.008, 0.1, 300.0)
    fake.receive.getActualQ()

    fake.freeze_stream()

    stamps = [fake.receive.getTimestamp() for _ in range(5)]
    positions = [fake.receive.getActualQ()[0] for _ in range(5)]
    assert stamps == [stamps[0]] * 5, f"the timestamp advanced while frozen: {stamps}"
    assert positions == [positions[0]] * 5, (
        f"the joint data advanced while frozen: {positions}")


def test_thaw_stream_lets_the_timestamp_and_the_joint_data_advance_again():
    fake = FakeRTDE(control_hz=125.0)
    fake.control.servoJ([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.008, 0.1, 300.0)
    fake.freeze_stream()
    fake.receive.getTimestamp()
    fake.receive.getActualQ()

    fake.thaw_stream()

    t0 = fake.receive.getTimestamp()
    t1 = fake.receive.getTimestamp()
    q0 = fake.receive.getActualQ()[0]
    q1 = fake.receive.getActualQ()[0]
    assert t1 - t0 == pytest.approx(1.0 / 125.0)
    assert q1 > q0
