"""Reviewed flight firmware and parameter invariants; no connection is initialized."""

from collections.abc import Mapping

from pymavlink.dialects.v10 import ardupilotmega as mavlink

DOWNWARD_ORIENTATION = mavlink.MAV_SENSOR_ROTATION_PITCH_270
PARAMETER_ABS_TOLERANCE = 1e-5

# These are the reviewed ArduCopter 4.7 invariants for this GPS-less aircraft.
# Hardware calibration and motor-output parameters deliberately do not belong here.
REQUIRED_NOGPS_LOITER_PARAMETERS: Mapping[str, float] = {
    "AHRS_EKF_TYPE": 3.0,
    "AHRS_OPTIONS": 16.0,
    "ARMING_NEED_LOC": 0.0,
    "AVOID_ENABLE": 2.0,
    "EK3_FLOW_USE": 1.0,
    "EK3_ENABLE": 1.0,
    "EK3_SRC1_POSXY": 0.0,
    "EK3_SRC1_POSZ": 1.0,
    "EK3_SRC1_VELXY": 5.0,
    "EK3_SRC1_VELZ": 0.0,
    "EK3_SRC1_YAW": 1.0,
    "EK3_SRC_OPTIONS": 0.0,
    "FLOW_TYPE": 5.0,
    "FRAME_CLASS": 1.0,
    "FRAME_TYPE": 1.0,
    "FS_CRASH_CHECK": 1.0,
    "FS_DR_ENABLE": 1.0,
    "FS_EKF_ACTION": 1.0,
    "FS_EKF_THRESH": 0.8,
    "FS_GCS_ENABLE": 5.0,
    "FS_GCS_TIMEOUT": 5.0,
    "FS_OPTIONS": 8.0,
    # This aircraft intentionally has no RC receiver.  In 4.7, disabling this
    # check is what permits full-check GCS arming without reporting "RC not found".
    "FS_THR_ENABLE": 0.0,
    "FS_VIBE_ENABLE": 1.0,
    "GPS1_TYPE": 0.0,
    "GPS2_TYPE": 0.0,
    # Bit 3 would reinterpret SET_ATTITUDE_TARGET's field as raw thrust.
    "GUID_OPTIONS": 0.0,
    # Preserve the project's deliberately gentle final descent rate.
    "LAND_SPD_MS": 0.15,
    "MAV_GCS_SYSID": 255.0,
    "RNGFND1_MAX": 1.0,
    "RNGFND1_ORIENT": float(DOWNWARD_ORIENTATION),
    "RNGFND1_TYPE": 10.0,
    # Fractional Guided climb setpoints are scaled by this value in 4.7.
    "WP_SPD_UP": 0.25,
}

# The optional forward MT-15 is independent of the downward altitude source.
# Preserve its reviewed conservative floor rather than its advertised wire minimum.
FORWARD_RANGEFINDER_PARAMETERS: Mapping[str, float] = {
    "RNGFND2_ORIENT": float(mavlink.MAV_SENSOR_ROTATION_NONE),
    "RNGFND2_MIN": 0.1,
    "RNGFND2_MAX": 15.0,
}

EXPECTED_FIRMWARE_VERSION = (4, 7, 1)
EXPECTED_FIRMWARE_COMMIT = b"dbe79216"
