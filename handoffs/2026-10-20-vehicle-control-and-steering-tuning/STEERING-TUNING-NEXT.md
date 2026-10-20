# 2026-10-20 — Next session: steering tuning

## Baseline to preserve

Normal marker: **STEER_LIMITS_20261019_R01**. Selected soft limits relative to physically verified straight zero: left **−8250 pulses**, right **+10000 pulses**. Positive pulses turn the front wheels right. STEP15, DIR16 inverted; rightward travel guard is GPIO18 (physically left switch), leftward guard GPIO17 (physically right switch). Both contacts were identified by manual holds; selected boundaries were chosen by the operator **before contact**, not by driving into the switches.

The diagnostic's old session coordinates left−14000 / center−5750 / right+4250 were relative to its temporary origin. Normal firmware re-expresses these around center0. Do not reuse old diagnostic absolute counts after a reset. Motor pulses depend on driver microstepping, gearbox, and linkage: record CL57T DIP/microstep settings and supply before comparing future runs. No actual wheel-angle degrees have been measured.

Current speed ceiling is 5000 pulses/s, acceleration 20000 pulses/s². The custom step method uses AccelStepper's actual direction, 10 µs direction setup, 100 µs STEP high, and returns STEP low. Smaller earlier bursts produced noise or invisible motion; a 5000-pulse burst visibly moved the linkage. Do not infer physical travel from a software counter alone.

## Sensor validation comes before automatic centering

The operator fitted an Oubit 12-bit, 360°, 0–5 V analog angle sensor and later reported an analog divider to A0/digital14. Divider resistor values were not recorded. Earlier BSS138 digital level conversion was unsuitable for this analog signal path. A0 must remain within the Teensy's analog input range; verify voltage at the input before powered calibration. A divider example is sensor OUT → 10 kΩ → A0, with 15 kΩ A0 → common GND (0.6 ratio); check actual supply/output limits and measured voltage rather than assuming the fitted divider matches this example.

In the last full-range diagnostic, **A0 median stayed approximately2500/4095 at center, right and left**, with only a few counts of noise. The operator confirmed both wheel motion and rotation of the sensor shaft. That is not a valid position map. The sensor is not proven dead; wiring, supply, ground, divider loading, wrong lead, or measurement path remain possible. See `evidence/sensor-retest-result.json` and the individual sensor-test records.

1. Keep motor power off for wiring/meter checks. Measure sensor supply and ground, then sensor OUT relative to sensor GND and A0 relative to Teensy GND. Rotate only with a safe isolated mechanical arrangement. Establish where the signal stops changing. Check continuity to **pin14**, divider junction, shared ground, and that the old digital converter is out of the analog path.
2. With the signal verified and safe voltage range measured, install the sensor diagnostic per DIAGNOSTICS.md. Confirm straight center explicitly, record stationary ADC, then make finite unloaded moves inside the saved limits. Compare ADC and a physical angle measurement at several positions in both directions; retain raw logs and direction/backlash observations.
3. Confirm the single-turn sensor remains unique across the entire steering range. A shaft making multiple turns or crossing the 0/360 discontinuity requires a suitable mapping/mechanical ratio or wrap-handling design; a single raw reading cannot distinguish repeated revolutions.
4. Reject rail readings, nearly equal left/center/right values, nonmonotonic traces, implausible spans, or inconsistent repeat runs. Measure backlash and missed steps; do not expand the current soft boundaries to compensate for a bad sensor.
5. Only after that, implement and test a stored profile (validity/version/CRC, monotonic endpoints, safe margins, out-of-range behavior) and explicit sensor-based referencing/centering. `steering_profile.h` and its regression are preparation; **normal steer.cpp does not currently load that profile or close its loop on A0**.

The desired future behavior is to restore a validated center on teleop startup and move there under controlled conditions. This remains pending. Do not just make Z automatic on startup: that declares an arbitrary current position straight. Preserve the operator-toggleable manual Z fallback (F2 in E-stop; `--disable-manual-reference` starts off) for encoder failure, with clear manual mode and reference validity.

## After sensor work

Tune rate/acceleration with unloaded wheels first; verify both directions and that normal targets stop before contacts, then test limit/fault/timeout handling without trapping the linkage. A limit currently clears reference and therefore causes teleop to inhibit the system; this can look like the whole car died. Determine whether it was a real switch event or an electrical reset from telemetry before changing that behavior. Use the C++ regressions for command/reference gating and physical observation for motion. Restore the normal steering image after temporary tests; retain original files and publish the next change as a new dated/titled handoff if continuing this additions-only convention.
