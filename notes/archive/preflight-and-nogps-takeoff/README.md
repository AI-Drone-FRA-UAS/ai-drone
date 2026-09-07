# August 2026 flight records — historical evidence

Imported on 2026-09-07 from `preflight-and-nogps-takeoff` at
`f1008308165b38fb770566b555020566d25ba183`. The original reports and logs are
preserved byte-for-byte; `manifest.json` records their original paths, lengths
and SHA-256 checksums. The reports are plain text because their commands,
parameters and descriptions of temporary flight code are historical evidence,
not current operating instructions.

| Record | Evidence |
| --- | --- |
| [20 August report](2026-08-20/original-report.txt) | Initial flight attempts, telemetry observations and provenance |
| [21 August report](2026-08-21/original-report.txt) | Accident investigation, repairs and subsequent lift/no-lift attempts |
| [Accident DataFlash log](2026-08-21/dataflash-log-2.bin) | Original FC log used in the investigation |
| [Overshoot telemetry](2026-08-21/alt-hold-takeoff-overshoot.tlog) | Historical ALT_HOLD attempt |
| [No-liftoff telemetry](2026-08-21/alt-hold-takeoff-no-liftoff.tlog) | Recorded estimator/accelerometer discrepancy |

These records describe the project aircraft at those dates. They do not prove
that the current mounting, calibration, firmware or sensor state is unchanged.
The incident's retired code and parameter snapshots remain reachable through
the archive tag and original Git commit; they are not activated in the maintained
package. Follow the current [flight procedure](../../../docs/PI_MAVLINK_CONTROL.md)
and latest `state/` evidence.
