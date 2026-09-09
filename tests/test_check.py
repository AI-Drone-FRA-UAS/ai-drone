from __future__ import annotations

import json
from collections import Counter, deque

import pytest

from ai_drone.cli import check

mavlink = check.mavlink


class Message:
    def __init__(self, kind, *, source=(1, 1), **fields):
        self.kind = kind
        self.source = source
        self.fields = fields
        self.__dict__.update(fields)

    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return self.source[0]

    def get_srcComponent(self):
        return self.source[1]

    def to_dict(self):
        return dict(self.fields)


class Clock:
    now = 0.0

    def monotonic(self):
        return self.now


class Connection:
    def __init__(self, clock):
        self.clock = clock
        self.mav = self
        self.pending = deque()
        self.sent = []
        self.closed = 0
        self.reads = Counter()
        self.parameters = check.expected_parameters()
        self.armed_phase = None
        self.source = (1, 1)
        self.streams = False
        self.cycle = 0
        self.missing = set()
        self.close_error = False
        self.prearm_result = mavlink.MAV_RESULT_ACCEPTED
        self.version = check.flight_config.EXPECTED_FIRMWARE_VERSION
        self.identity = check.flight_config.EXPECTED_FIRMWARE_COMMIT
        mask = sum(check.HEALTH_BITS.values()) | mavlink.MAV_SYS_STATUS_PREARM_CHECK
        self.data = [
            Message(
                "SYS_STATUS",
                voltage_battery=15100,
                onboard_control_sensors_present=mask,
                onboard_control_sensors_enabled=mask,
                onboard_control_sensors_health=mask,
            ),
            Message(
                "RAW_IMU",
                xacc=0,
                yacc=0,
                zacc=-1000,
                xgyro=0,
                ygyro=0,
                zgyro=0,
                xmag=200,
                ymag=-50,
                zmag=600,
            ),
            Message("ATTITUDE", roll=0.0, pitch=0.0, yaw=0.2),
            Message(
                "DISTANCE_SENSOR",
                orientation=25,
                id=0,
                current_distance=2,
                min_distance=2,
                max_distance=100,
            ),
            Message(
                "DISTANCE_SENSOR",
                orientation=0,
                id=1,
                current_distance=150,
                min_distance=10,
                max_distance=1500,
            ),
            Message("OPTICAL_FLOW", quality=60, flow_comp_m_x=0.0, flow_comp_m_y=0.0),
            Message("SCALED_PRESSURE", press_abs=998.0),
            Message(
                "EKF_STATUS_REPORT",
                flags=mavlink.EKF_ATTITUDE | mavlink.EKF_CONST_POS_MODE,
            ),
            Message("RC_CHANNELS", chancount=0),
        ]

    def heartbeat(self):
        armed = (
            self.armed_phase == "initial"
            or (self.armed_phase == "parameters" and self.pending)
            or (self.armed_phase == "capture" and self.streams)
        )
        return Message(
            "HEARTBEAT",
            source=self.source,
            base_mode=mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
            autopilot=mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            type=mavlink.MAV_TYPE_QUADROTOR,
        )

    def command_long_send(self, *values):
        self.sent.append(("command_long", values))
        command = values[2]
        assert command in {
            mavlink.MAV_CMD_REQUEST_MESSAGE,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            mavlink.MAV_CMD_RUN_PREARM_CHECKS,
        }, "Any actuation/mode/arming command is forbidden"
        assert values[:2] == (1, 1)
        if command == mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
            self.streams = True
            assert values[4] in {
                getattr(mavlink, f"MAVLINK_MSG_ID_{name}") for name in check.RATES_HZ
            }
            assert values[5] in (200000, 500000)
        elif command == mavlink.MAV_CMD_REQUEST_MESSAGE:
            assert values[4] == mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION
            packed = sum(
                value << shift
                for value, shift in zip(self.version, (24, 16, 8), strict=True)
            )
            self.pending.append(
                Message(
                    "AUTOPILOT_VERSION",
                    source=self.source,
                    flight_sw_version=packed,
                    flight_custom_version=self.identity,
                )
            )
        else:
            assert values[4:] == (0.0, 0, 0, 0, 0, 0, 0)
            if self.prearm_result is not None:
                self.pending.append(
                    Message(
                        "COMMAND_ACK",
                        command=mavlink.MAV_CMD_RUN_PREARM_CHECKS,
                        result=self.prearm_result,
                    )
                )

    def param_request_read_send(self, system, component, name, index):
        assert (system, component, index) == (1, 1, -1)
        self.sent.append(("param_read", name.decode()))
        self.reads[name.decode()] += 1
        if name.decode() in self.parameters:
            self.pending.append(
                Message(
                    "PARAM_VALUE",
                    source=self.source,
                    param_id=name,
                    param_value=self.parameters[name.decode()],
                )
            )

    def recv_match(self, *, blocking, timeout=0, type=None):
        if not blocking:
            return None
        self.clock.now += min(0.01, timeout)
        if type == "HEARTBEAT" or self.armed_phase:
            return self.heartbeat()
        if self.pending:
            return self.pending.popleft()
        self.cycle += 1
        if self.cycle % (len(self.data) + 1) == 0:
            return self.heartbeat()
        message = self.data[(self.cycle - 1) % (len(self.data) + 1)]
        message.source = self.source
        return None if message.kind in self.missing else message

    def close(self):
        self.closed += 1
        if self.close_error:
            raise OSError("close failed")


@pytest.fixture
def rig(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(check.time, "monotonic", clock.monotonic)
    connection = Connection(clock)
    monkeypatch.setattr(
        check, "_resolve_check_endpoint", lambda *_a, **_k: "/dev/test-fc"
    )
    monkeypatch.setattr(check, "require_available_serial", lambda *_a, **_k: None)
    monkeypatch.setattr(
        check, "open_ardupilot_connection", lambda *_a, **_k: connection
    )

    def forbidden_controller(*_args, **_kwargs):
        pytest.fail("Bench check must never create a DroneController")

    monkeypatch.setattr(
        check.flight_config.DroneController, "__init__", forbidden_controller
    )
    return clock, connection


def execute(capsys, *extra):
    code = check.main(
        [
            "--duration",
            "0.3",
            "--timeout",
            "0.2",
            "--parameter-timeout",
            "1",
            "--json",
            *extra,
        ]
    )
    return code, json.loads(capsys.readouterr().out)


def test_check_is_read_only_and_bench_aiding_is_not_flight_clearance(rig, capsys):
    _, connection = rig
    code, report = execute(capsys)
    assert code == 0
    assert report["passed"] is True
    assert connection.closed == 1
    assert report["sensors"]["battery_v"] == 15.1
    assert report["sensors"]["forward_range"]["distance_m"] == 1.5
    assert report["sensors"]["downward_range"]["distance_m"] == 0.02
    assert report["sensors"]["fc_health"]["compass"]["health"] is True
    assert report["sensors"]["ekf"]["relative_position"] is False
    assert any("Relative EKF aiding" in text for text in report["warnings"])
    assert "ROMFS" in report["limitation"]
    assert set(connection.reads) == set(check.expected_parameters())
    assert all(count == 1 for count in connection.reads.values())
    assert all(
        values[2] != mavlink.MAV_CMD_RUN_PREARM_CHECKS
        for kind, values in connection.sent
        if kind == "command_long"
    )


@pytest.mark.parametrize("phase", ["initial", "parameters", "capture"])
def test_armed_at_any_phase_aborts_and_only_closes(rig, capsys, phase):
    _, connection = rig
    connection.armed_phase = phase
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 1
    assert any("ARMED" in error for error in report["errors"])
    if phase == "initial":
        assert connection.sent == []


def test_other_vehicle_cannot_select_target_or_trigger_requests(rig, capsys):
    _, connection = rig
    connection.source = (2, 1)
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 1
    assert connection.sent == []
    assert report["messages"] == {}


def test_missing_parameter_phase_has_absolute_deadline_and_one_retry(rig, capsys):
    clock, connection = rig
    del connection.parameters["ARMING_SKIPCHK"]
    code, report = execute(capsys)
    assert code == 1
    assert "ARMING_SKIPCHK: parameter unavailable" in report["errors"]
    assert 1.3 <= clock.now < 1.6
    assert connection.reads["ARMING_SKIPCHK"] == 2
    assert max(connection.reads.values()) == 2
    assert connection.closed == 1


@pytest.mark.parametrize(
    "name,value",
    [
        ("ARMING_SKIPCHK", 1),
        ("RNGFND2_TYPE", 0),
        ("RNGFND2_ORIENT", 25),
        ("GPS1_TYPE", 1),
        ("EK3_SRC1_POSXY", float("nan")),
    ],
)
def test_configuration_mismatch_or_nonfinite_parameter_fails(rig, capsys, name, value):
    _, connection = rig
    connection.parameters[name] = value
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 1
    assert any(name in error for error in report["errors"])


def test_firmware_expectations_follow_active_gate_without_duplicate_pin(
    rig, capsys, monkeypatch
):
    _, connection = rig
    monkeypatch.setattr(check.flight_config, "EXPECTED_FIRMWARE_VERSION", (4, 7, 1))
    monkeypatch.setattr(check.flight_config, "EXPECTED_FIRMWARE_COMMIT", b"dbe79216")
    connection.version, connection.identity = (4, 7, 1), b"dbe79216"
    code, report = execute(capsys)
    assert code == 0
    assert report["firmware"]["expected_identity"] == "dbe79216"


def test_wrong_firmware_identity_fails(rig, capsys):
    _, connection = rig
    connection.identity = b"deadbeef"
    code, report = execute(capsys)
    assert code == 1 and not report["firmware"]["matches"]
    assert connection.closed == 1


@pytest.mark.parametrize(
    "missing",
    [
        "RAW_IMU",
        "SCALED_PRESSURE",
        "OPTICAL_FLOW",
        "DISTANCE_SENSOR",
        "EKF_STATUS_REPORT",
        "RC_CHANNELS",
    ],
)
def test_missing_required_sensor_stream_fails(rig, capsys, missing):
    _, connection = rig
    connection.missing.add(missing)
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 1
    assert any(missing in text for text in report["errors"])


def test_sensor_receipt_does_not_override_fc_health(rig, capsys):
    _, connection = rig
    connection.data[0].fields[
        "onboard_control_sensors_health"
    ] &= ~mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG
    code, report = execute(capsys)
    assert code == 1
    assert report["messages"]["RAW_IMU"] > 0
    assert not report["sensors"]["fc_health"]["compass"]["health"]


def test_low_battery_and_detected_receiver_fail(rig, capsys):
    _, connection = rig
    connection.data[0].fields["voltage_battery"] = 13000
    connection.data[-1].fields["chancount"] = 16
    code, report = execute(capsys)
    assert code == 1
    assert any("Battery" in error for error in report["errors"])
    assert any("RC channel" in error for error in report["errors"])


def test_below_range_floor_is_reported_without_demanding_hand_lift(rig, capsys):
    _, connection = rig
    connection.data[3].fields["min_distance"] = 10
    code, report = execute(capsys)
    assert code == 0
    assert report["sensors"]["downward_range"]["within_reported_bounds"] is False
    assert any("bench geometry" in text for text in report["warnings"])


@pytest.mark.parametrize(
    "index,field,value,expected",
    [
        (4, "id", 0, "expected FC instance"),
        (4, "current_distance", 65535, "no finite usable distance"),
        (5, "flow_comp_m_x", float("nan"), "invalid compensated velocity"),
        (6, "press_abs", float("nan"), "invalid pressure"),
    ],
)
def test_invalid_sensor_values_cannot_pass_on_receipt_alone(
    rig, capsys, index, field, value, expected
):
    _, connection = rig
    connection.data[index].fields[field] = value
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 1
    assert any(expected in text for text in report["errors"])


def test_freshness_rejects_an_old_sample_despite_positive_message_count(rig):
    clock, connection = rig
    observed = check.Observations()
    observed.observe(connection.data[5])
    clock.now += check.FRESHNESS_S + 0.1
    assert observed.counts["OPTICAL_FLOW"] == 1
    assert observed.fresh("OPTICAL_FLOW", clock.now) is None
    assert observed.errors == ["OPTICAL_FLOW: missing or stale FC telemetry"]


def test_parameter_transport_error_still_only_closes(rig, capsys, monkeypatch):
    _, connection = rig

    def failed_read(*_args):
        raise OSError("parameter transport failed")

    monkeypatch.setattr(connection, "param_request_read_send", failed_read)
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 1
    assert "parameter transport failed" in report["errors"]


def test_optional_prearm_uses_only_diagnostic_command(rig, capsys):
    _, connection = rig
    connection.data.append(Message("STATUSTEXT", text="PreArm: Compass inconsistent"))
    code, report = execute(capsys, "--prearm")
    assert code == 1
    assert report["prearm_requested"] is True
    assert any("Compass inconsistent" in text for text in report["errors"])
    assert (
        sum(
            kind == "command_long" and values[2] == mavlink.MAV_CMD_RUN_PREARM_CHECKS
            for kind, values in connection.sent
        )
        == 1
    )


def test_close_failure_prevents_success(rig, capsys):
    _, connection = rig
    connection.close_error = True
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 1
    assert "Closing connection failed: close failed" in report["errors"]


@pytest.mark.parametrize("result", [None, mavlink.MAV_RESULT_UNSUPPORTED])
def test_prearm_request_missing_or_negative_ack_is_visible(rig, capsys, result):
    _, connection = rig
    connection.prearm_result = result
    code, report = execute(capsys, "--prearm")
    assert code == 1
    assert report["prearm_command_result"] == result
    assert any(
        "diagnostic request was not accepted" in text for text in report["errors"]
    )


@pytest.mark.parametrize(
    "option,value",
    [
        ("--duration", "nan"),
        ("--parameter-timeout", "inf"),
        ("--timeout", "0"),
        ("--baud", "0"),
        ("--min-battery", "-1"),
    ],
)
def test_invalid_limits_never_open_connection(rig, capsys, option, value):
    _, connection = rig
    with pytest.raises(SystemExit) as error:
        execute(capsys, option, value)
    assert error.value.code == 2
    assert connection.closed == 0 and not connection.sent


def test_command_allowlist_rejects_actuation_without_sending(rig):
    _, connection = rig
    with pytest.raises(ValueError, match="allowlist"):
        check._command(connection, mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
    assert not connection.sent


def test_busy_guard_runs_before_connection_open_or_baud_change(
    rig, capsys, monkeypatch
):
    _, connection = rig

    def busy(*_args, **_kwargs):
        raise RuntimeError("Serial device is busy")

    def never_open(*_args, **_kwargs):
        pytest.fail("Busy endpoint must not be opened, even briefly")

    monkeypatch.setattr(check, "require_available_serial", busy)
    monkeypatch.setattr(check, "open_ardupilot_connection", never_open)
    code, report = execute(capsys)
    assert code == 1 and connection.closed == 0 and connection.sent == []
    assert report["errors"] == ["Serial device is busy"]


@pytest.mark.parametrize(
    "usb,pi,uart,expected",
    [
        (True, True, True, "usb"),
        (False, True, True, "/dev/serial0"),
        (False, False, True, None),
        (False, True, False, None),
    ],
)
def test_default_discovery_never_guesses_other_usb_aircraft(
    monkeypatch, usb, pi, uart, expected
):
    stable = str(check.STABLE_FLIGHT_CONTROLLER_DEVICE)
    existing = {"/dev/ttyACM0", "/dev/serial/by-id/usb-ArduPilot_OTHER-if00"}
    if usb:
        existing.add(stable)
    if uart:
        existing.add("/dev/serial0")
    monkeypatch.setattr(check.Path, "exists", lambda path: str(path) in existing)
    monkeypatch.setattr(check, "is_raspberry_pi", lambda: pi)

    def never_guess(*_args, **_kwargs):
        pytest.fail("Generic serial discovery cannot select the default aircraft")

    monkeypatch.setattr(check, "resolve_mavlink_endpoint", never_guess)
    if expected is None:
        with pytest.raises(FileNotFoundError, match="explicitly"):
            check._resolve_check_endpoint(None)
    else:
        assert check._resolve_check_endpoint(None) == (
            stable if expected == "usb" else expected
        )


def test_other_endpoint_requires_and_respects_explicit_selection(monkeypatch):
    seen = []
    monkeypatch.setattr(
        check, "resolve_mavlink_endpoint", lambda value: seen.append(value) or value
    )
    assert check._resolve_check_endpoint("tcp:127.0.0.1:5760") == "tcp:127.0.0.1:5760"
    assert seen == ["tcp:127.0.0.1:5760"]
