# 2026-10-20 — Validation and evidence

## Publication checks

The dated source snapshot was checked without contacting or moving vehicle hardware:

- All four Teensy4.1 normal environments built: master, throttle, steer, brake. PlatformIO6.1.19; Teensy platform5.1.0; Arduino Teensy framework1.160.0; ARM GCC11.3.1; uploader1.162.0.
- Sixteen Python teleop tests passed. Coverage includes startup/E-stop, manual-reference toggle/gates, missing/stale CAN feedback, direction reversal, shutdown and active steering.
- Twelve host C++ regression executables passed using actual firmware included against mocks: master, steering profile, limit diagnostic, sensor diagnostic, watchdog clock/rollover, four-node CAN, CAN pair, normal master/steering pair, USB jog, normal steer, link sender and responder. Host compiler was the installed MinGW GCC5.1 toolchain. Fixture unused-variable warnings do not represent test failures.
- Setup created a fresh Python3.11 virtual environment and installed the pinned requirements. The documented Test-Package command was exercised with that environment.
- Diagnostic build and script/hash results are recorded in `evidence/publication-validation.json`; SHA256s for every delivered file and image are in `manifest.json`. The manifest excludes itself, virtual environments, caches, build outputs and runtime logs. Nested .gitattributes preserves archived bytes across Windows Git checkouts.

These checks establish buildability and tested software behavior. They do not establish loaded vehicle safety, electrical continuity or actual wheel position. The publication process performed **no firmware upload or vehicle movement**. Packaged installation records are historical evidence from this car, not proof that a reader's board already has that image installed. Newly built HEX files can differ from preserved historical HEX files due to build metadata/configuration; archived images are hash-identified separately.

## Prior hardware evidence retained

| Evidence | What it establishes | Limits |
|---|---|---|
| `master-can-20261016-221847Z.log` | At that session, all three CAN diagnostic responders were OK with increasing replies and reported TEC/REC/fault0 | Temporary CAN-test firmware; not a new electrical/actuator check |
| `selected-boundaries.json`, cited limit-test logs | Operator selected soft boundaries and confirmed straight center; relative motor counts recorded | Neither physical switch was reached; no wheel-angle degrees or independent alignment measurement |
| `physical-observations.json` | Positive motion was observed rightward, toward physically left-mounted switch | Earlier-stage observations; read selected-boundaries for later confirmed center |
| `sensor-retest-result.json`, five stationary sensor-test logs | Center/right/left 12-bit A0 medians stayed about2500 while commanded steps changed | Sensor position mapping invalid; actual electrical cause unresolved |
| `stationary-20261020-050116Z.log` and installation records | Normal steering marker and selected limits seen after restoring operational firmware | Passive stationary check, not a new full physical sweep |
| `steering-master-test-status.json` and matching JSONL | Normal master CAN path reached right/center/left/center software targets; all node feedback fresh; final E-stop acknowledged, reference retained | Status explicitly says physical movement AWAITING_OPERATOR_OBSERVATION; no full-sequence physical confirmation recorded in this log |

The master test began 2026-10-20 05:04:28UTC (October19 evening in Pacific time). This handoff is dated October20 for the current publication request. Throttle was zero, gearN and commanded brake1 throughout that steering-only test.

Operator chat subsequently reported working full-car control/acceleration before requesting steering tuning. That report is retained as context, not upgraded to a measured acceleration/braking performance result. Brake feedback constants and successful software/CAN tests do not establish loaded stopping force. Earlier missing-throttle CHECK-CAN screens and failed traction tests were earlier states, not the final reported behavior.

## Source provenance

This adds a dated copy under an unchanged repository base; it does not replace root firmware or root documentation. `source-provenance.json` identifies the original local hashes. Current master.cpp, throttle.cpp, brake.cpp and dbw_can.h matched their preserved R02 source archive byte-for-byte. Steering is the newer mapped-limits source. New-copy adaptations pin dependencies, use relative/local-venv paths, route logs to runtime, require actuator-power confirmation for upload, and restore the newer steering baseline from the legacy uploader. Existing local repository changes were not staged into existing paths.

Future development should use this dated firmware/teleop pair or a new dated successor. Current analog feedback, profile persistence, automatic reference/centering, actual wheel-angle calibration, driver DIP settings and loaded braking remain distinct follow-up work. Next steps are in STEERING-TUNING-NEXT.md.
