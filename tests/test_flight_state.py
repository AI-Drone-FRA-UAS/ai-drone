from dataclasses import replace
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v10 import ardupilotmega as mavlink

from ai_drone.flight.state import (
    AltitudeAlignment,
    BootClock,
    Heartbeat,
    Observation,
    Reading,
    Sample,
    TimedReading,
    VehicleState,
    advance_boot,
    decode,
    fresh,
    observe,
)


def packet(kind, **fields):
    return SimpleNamespace(
        get_type=lambda: kind,
        get_srcSystem=lambda: 1,
        get_srcComponent=lambda: 1,
        **fields,
    )


def apply(state, payload, received=100.0, now=100.0):
    return observe(state, Observation(1, 1, received, payload), now=now)


@pytest.mark.parametrize(
    "age,valid", [(0, True), (1, True), (1.0001, False), (-0.001, False)]
)
def test_freshness_is_inclusive_and_rejects_future_receipts(age, valid):
    assert (fresh(Sample(42, 100.0 - age), 100.0, 1.0) == 42) is valid


@pytest.mark.parametrize("received", [97.499, 100.001, float("nan"), float("inf")])
def test_heartbeat_rejects_stale_future_and_nonfinite_receipts(received):
    before = VehicleState(heartbeat=Sample(Heartbeat(True, "LOITER"), 99.0))
    assert apply(before, Heartbeat(False, "LAND"), received) is before


def test_heartbeat_uses_original_receipt_and_rejects_older_selected_packets():
    state = apply(VehicleState(), Heartbeat(True, "LOITER"), 97.5)
    assert state.heartbeat == Sample(Heartbeat(True, "LOITER"), 97.5)
    assert apply(state, Heartbeat(False, "LAND"), 97.4) is state


def test_foreign_source_cannot_update_state():
    before = VehicleState()
    for system, component in ((2, 1), (1, 2)):
        event = Observation(system, component, 100.0, Heartbeat(True, "LOITER"))
        assert observe(before, event, now=100.0) is before


def test_heartbeat_mode_fallback_and_missing_mode_preserve_last_known_mode():
    event = decode(
        packet("HEARTBEAT", base_mode=0), received=100.0, fallback_mode="LAND"
    )
    assert event is not None
    state = observe(VehicleState(), event, now=100.0)
    assert state.heartbeat == Sample(Heartbeat(False, "LAND"), 100.0)
    state = apply(state, Heartbeat(False, None), 100.0)
    assert state.heartbeat == Sample(Heartbeat(False, "LAND"), 100.0)


@pytest.mark.parametrize("raw", [None, True, 0, -1, 2**32, 100.0, "100"])
def test_invalid_boot_timestamps_never_establish_clock(raw):
    assert advance_boot(None, raw, 100.0) is None


@pytest.mark.parametrize(
    "raw,accepted", [(9_100, True), (9_099, False), (10_350, True), (10_351, False)]
)
def test_boot_age_tolerances_remain_inclusive(raw, accepted):
    assert (advance_boot(BootClock(10_000, 100.0), raw, 100.1) is not None) is accepted


def test_boot_time_wrap_preserves_freshness():
    before = BootClock(2**32 - 50, 100.0)
    assert advance_boot(before, 50, 100.1) == BootClock(50, 100.1)


def test_stale_and_future_downward_samples_do_not_replace_range():
    state = apply(VehicleState(), TimedReading("altitude", 0.4, 10_000))
    for milliseconds, received in ((8_000, 100.1), (15_000, 100.2)):
        state = apply(
            state, TimedReading("altitude", 0.9, milliseconds), received, received
        )
    assert state.altitude == Sample(0.4, 100.0)


def test_only_downward_range_decodes_as_altitude():
    fields = dict(
        time_boot_ms=1_000,
        current_distance=42,
        min_distance=2,
        max_distance=100,
        signal_quality=0,
    )
    assert (
        decode(packet("DISTANCE_SENSOR", orientation=0, **fields), received=100.0)
        is None
    )
    event = decode(
        packet(
            "DISTANCE_SENSOR",
            orientation=mavlink.MAV_SENSOR_ROTATION_PITCH_270,
            **fields,
        ),
        received=100.0,
    )
    assert event is not None
    assert observe(VehicleState(), event, now=100.0).altitude == Sample(0.42, 100.0)


@pytest.mark.parametrize("voltage", [0, -1, 65_535, 65_536, float("nan"), float("inf")])
def test_invalid_battery_report_revokes_previous_good_sample(voltage):
    fields = dict.fromkeys(
        (
            "onboard_control_sensors_present",
            "onboard_control_sensors_enabled",
            "onboard_control_sensors_health",
        ),
        mavlink.MAV_SYS_STATUS_SENSOR_BATTERY,
    )
    event = decode(
        packet("SYS_STATUS", voltage_battery=voltage, **fields), received=100.0
    )
    assert event is not None
    before = VehicleState(battery=Sample(16.0, 99.9))
    assert observe(before, event, now=100.0).battery is None


@pytest.mark.parametrize(
    "field",
    [
        "onboard_control_sensors_present",
        "onboard_control_sensors_enabled",
        "onboard_control_sensors_health",
    ],
)
def test_battery_health_must_be_present_enabled_and_healthy(field):
    fields = dict.fromkeys(
        (
            "onboard_control_sensors_present",
            "onboard_control_sensors_enabled",
            "onboard_control_sensors_health",
        ),
        mavlink.MAV_SYS_STATUS_SENSOR_BATTERY,
    )
    fields[field] = 0
    event = decode(
        packet("SYS_STATUS", voltage_battery=16_000, **fields), received=100.0
    )
    assert event is not None
    state = observe(VehicleState(battery=Sample(16.0, 99.9)), event, now=100.0)
    assert state.battery is None
    assert apply(state, Reading("battery", 15.9)).battery == Sample(15.9, 100.0)


def test_zero_quality_revokes_flow_validity_without_fabricating_good_data():
    state = VehicleState(flow_quality=Sample(50, 99.9))
    assert apply(state, Reading("flow_quality", 0)).flow_quality == Sample(0, 100.0)


def test_alignment_keeps_range_and_local_datums_distinct_and_resets_on_disarm():
    state = apply(VehicleState(), TimedReading("altitude", 0.05, 1_000))
    state = apply(state, TimedReading("local_altitude", 12.0, 1_010))
    state = apply(state, TimedReading("local_altitude", 12.8, 1_510), 100.5, 100.5)
    assert state.local_altitude == Sample(12.8, 100.5)
    assert state.altitude == Sample(0.05, 100.0)
    assert state.alignment is not None
    assert state.alignment.local.value == pytest.approx(0.85)
    disarmed = apply(state, Heartbeat(False, "LAND"), 100.5, 100.5)
    assert disarmed.alignment is None
    assert disarmed.local_altitude == state.local_altitude
    assert disarmed.altitude == state.altitude


def test_alignment_does_not_reanchor_from_stale_range():
    state = VehicleState(altitude=Sample(0.05, 98.0))
    assert apply(state, TimedReading("local_altitude", 12.0, 1_000)).alignment is None
    state = replace(state, alignment=AltitudeAlignment(-11.95, Sample(0.05, 98.0)))
    assert (
        apply(state, TimedReading("local_altitude", 12.2, 1_000)).alignment
        is state.alignment
    )


@pytest.mark.parametrize(
    "kind,value",
    [("altitude", 0.4), ("local_altitude", 0.4), ("yaw", 0.2), ("rc_channels", 0)],
)
def test_invalid_current_timed_sample_revokes_old_good_evidence(kind, value):
    state = apply(VehicleState(), TimedReading(kind, value, 1_000))
    state = apply(state, TimedReading(kind, None, 1_010), 100.01, 100.01)
    assert getattr(state, kind) is None
    # A delayed older valid packet cannot undo the invalidation.
    assert apply(state, TimedReading(kind, value, 1_005), 100.005, 100.02) is state
    recovered = apply(state, TimedReading(kind, value, 1_020), 100.02, 100.02)
    assert getattr(recovered, kind) == Sample(value, 100.02)


def test_replayed_same_source_time_cannot_refresh_range_or_erase_it():
    state = apply(VehicleState(), TimedReading("altitude", 0.4, 1_000))
    for value in (0.6, None):
        assert (
            apply(state, TimedReading("altitude", value, 1_000), 100.1, 100.1) is state
        )
    assert state.altitude == Sample(0.4, 100.0)


def test_cross_stream_clock_advance_does_not_make_an_older_range_current():
    state = apply(VehicleState(), TimedReading("altitude", 0.4, 1_000))
    state = apply(state, TimedReading("yaw", 0.2, 1_100), 100.1, 100.1)
    state = apply(state, TimedReading("altitude", 0.5, 1_050), 100.15, 100.15)
    assert state.altitude == Sample(0.5, 100.15)
    assert apply(state, TimedReading("altitude", 0.8, 1_020), 100.2, 100.2) is state


@pytest.mark.parametrize(
    "payload",
    [
        Reading("flow_quality", 60),
        Reading("battery", 16.0),
        TimedReading("yaw", 0.1, 1_000),
    ],
)
@pytest.mark.parametrize("received", [100.001, float("nan"), float("inf")])
def test_no_payload_type_can_install_future_or_nonfinite_receipt_time(
    payload, received
):
    state = VehicleState()
    assert apply(state, payload, received) is state


def test_out_of_order_flow_cannot_restore_revoked_quality():
    state = apply(VehicleState(), Reading("flow_quality", 60), 99.0)
    state = apply(state, Reading("flow_quality", 0), 99.5)
    assert apply(state, Reading("flow_quality", 60), 99.2) is state
    assert state.flow_quality == Sample(0, 99.5)


@pytest.mark.parametrize("identifier", [1, 2, 255, False])
def test_other_downward_instance_cannot_replace_selected_range(identifier):
    assert (
        decode(
            packet(
                "DISTANCE_SENSOR",
                id=identifier,
                orientation=25,
                time_boot_ms=1_000,
                current_distance=45,
                min_distance=2,
                max_distance=100,
            ),
            received=100.0,
        )
        is None
    )
