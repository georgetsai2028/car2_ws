"""Bounded 12-bit A0 retest. Requires STEER_SENSOR_TEST_V1; no analog position control."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time
import statistics

import serial
from serial.tools import list_ports

IDENTITY = "20369970"
MARKER = "STEER_SENSOR_TEST_V1"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--steps", type=int, default=0,
                        help="0 reads only; -5000..5000 requests ONE finite burst")
    parser.add_argument("--expected-position", type=int,
                        help="Required for motion; refuse if the saved test reference has changed")
    parser.add_argument("--zero", action="store_true", help="Record physically confirmed straight position; no movement")
    parser.add_argument("--center-confirmed", action="store_true")
    parser.add_argument("--clear-fault", action="store_true", help="Clear only with valid ADC and released switches")
    parser.add_argument("--seconds", type=float, default=2)
    args = parser.parse_args()
    if not -5000 <= args.steps <= 5000:
        parser.error("steps must be within -5000..5000")
    if args.steps and args.expected_position is None:
        parser.error("motion requires --expected-position to guard the calibration reference")
    if args.zero and (not args.center_confirmed or args.steps or args.clear_fault):
        parser.error("ZERO requires --center-confirmed and cannot be combined with motion or fault clear")
    if args.clear_fault and args.steps:
        parser.error("fault clear cannot be combined with motion")
    if not 1 <= args.seconds <= 15:
        parser.error("stationary seconds must be within 1..15")
    matches = [p for p in list_ports.comports() if p.device.upper() == args.port.upper()]
    if (len(matches) != 1 or matches[0].serial_number != IDENTITY or
            matches[0].vid != 0x16C0 or matches[0].pid != 0x0483):
        raise RuntimeError(f"Expected steering USB serial {IDENTITY} on {args.port}")
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d-%H%M%SZ")
    runtime = Path(__file__).resolve().parents[2] / "runtime/steering"
    runtime.mkdir(parents=True, exist_ok=True)
    destination = runtime / f"sensor-test-{stamp}.json"
    records = []
    result = {"port": args.port, "serial": IDENTITY, "requestedSteps": args.steps,
              "frames": records, "physicalMotionConfirmed": False}
    with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=0.2) as link:
        def send(command):
            link.write((command + "\n").encode("ascii"))

        def read():
            line = link.readline().decode("ascii", errors="replace").strip()
            if not line:
                return None
            try:
                state = json.loads(line)
            except json.JSONDecodeError:
                raise RuntimeError(f"Unexpected firmware output: {line[:160]}")
            if state.get("fw") != MARKER:
                raise RuntimeError("Wrong firmware marker; no jog allowed")
            if state.get("pulseUs") != 100 or state.get("maxBurst") != 5000:
                raise RuntimeError("Unexpected diagnostic pulse configuration; no jog allowed")
            if state.get("rightStopPin") != 18 or state.get("leftStopPin") != 17:
                raise RuntimeError("Unexpected steering travel/switch mapping; no jog allowed")
            if state.get("leftBound") != -8250 or state.get("rightBound") != 10000:
                raise RuntimeError("Unexpected saved soft bounds; no jog allowed")
            records.append({"utc": dt.datetime.now(dt.timezone.utc).isoformat(), **state})
            return state

        def wait_for(predicate, seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                state = read()
                if state is not None and predicate(state):
                    return state
            raise RuntimeError("Timed out awaiting diagnostic acknowledgement")

        try:
            # Verify firmware before sending even a non-motion command.
            wait_for(lambda s: True, 3)
            link.reset_input_buffer()
            send("STOP")
            send("P")
            initial = wait_for(lambda s: not s["moving"] and not s["armed"] and
                               s["event"] in ("USB_STOP", "SENSOR_RAIL_FAULT", "BOTH_LIMITS_FAULT"), 2)
            if args.zero or args.clear_fault:
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    read()
                if args.zero and initial["fault"]:
                    raise RuntimeError("Fault remains; no center recorded")
                action = "ZERO" if args.zero else "CLEAR"
                send(action)
                send("P")
                response = wait_for(lambda s: s["event"] != "USB_STOP", 1)
                if args.zero and (response["event"] != "MANUAL_ZERO" or not response["manualZero"]):
                    raise RuntimeError("Firmware refused straight-center reference")
                if args.clear_fault and response["fault"]:
                    raise RuntimeError("Fault remains after clear request")
                result["action"] = action
            if not args.steps:
                deadline = time.monotonic() + args.seconds
                while time.monotonic() < deadline:
                    read()
                result["outcome"] = "STATIONARY_ONLY"
            else:
                if initial["pos"] != args.expected_position:
                    raise RuntimeError("Test position changed; refusing to move with an invalid reference")
                if not initial["manualZero"]:
                    raise RuntimeError("No operator-confirmed straight reference; no jog")
                if not -8250 <= initial["pos"] + args.steps <= 10000:
                    raise RuntimeError("Requested move exceeds the saved travel bounds")
                if initial["fault"] or (args.steps > 0 and initial["rightTravelLimit"]) or (
                        args.steps < 0 and initial["leftTravelLimit"]):
                    raise RuntimeError("Fault or requested-direction limit asserted; no jog")
                # Allow the firmware's switch stability requirement to settle.
                deadline = time.monotonic() + 0.6
                while time.monotonic() < deadline:
                    read()
                link.reset_input_buffer()
                send("ARM")
                send("P")
                armed = wait_for(lambda s: s["event"] in ("ARMED", "ARM_REFUSED"), 1)
                if not armed["armed"]:
                    raise RuntimeError("Firmware refused arm; no jog")
                if armed["pos"] != args.expected_position:
                    raise RuntimeError("Test position changed during arming; no jog")
                initial_pos = armed["pos"]
                send(f"J {args.steps}")
                deadline = time.monotonic() + 3.2
                next_keepalive = 0
                final = None
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if now >= next_keepalive:
                        send("K")
                        next_keepalive = now + 0.05
                    state = read()
                    if state is None:
                        continue
                    if state["fault"]:
                        raise RuntimeError(f"Firmware fault: {state['event']}")
                    if not state["moving"] and state["event"] not in ("ARMED", "USB_STOP"):
                        final = state
                        break
                if final is None:
                    raise RuntimeError("Host move deadline exceeded")
                result.update(outcome=final["event"], initialPosition=initial_pos,
                              finalPosition=final["pos"], commandedStepChange=final["pos"] - initial_pos)
                if final["event"] not in ("BURST_DONE", "RIGHT_LIMIT_STOP", "LEFT_LIMIT_STOP"):
                    raise RuntimeError(f"Burst stopped: {final['event']}")
            values = [state["adc12"] for state in records]
            result["adc12"] = {"min": min(values), "max": max(values),
                               "median": statistics.median(values),
                               "stdev": round(statistics.pstdev(values), 2), "samples": len(values)}
            print(json.dumps({key: value for key, value in result.items() if key != "frames"}))
            print("Last status:", json.dumps(records[-1]))
        except BaseException as error:
            result["error"] = str(error)
            raise
        finally:
            try:
                send("STOP")
            finally:
                destination.write_text(json.dumps(result, indent=2), encoding="utf-8")
                print("Log:", destination)


if __name__ == "__main__":
    main()
