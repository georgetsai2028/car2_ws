"""One finite steering burst, or stationary status. Requires STEER_LIMIT_TEST_V3."""
import argparse
import datetime as dt
import json
from pathlib import Path
import time

import serial
from serial.tools import list_ports

IDENTITY = "20369970"
MARKER = "STEER_LIMIT_TEST_V3"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM5")
    parser.add_argument("--steps", type=int, default=0,
                        help="0 reads only; -5000..5000 requests ONE finite burst")
    parser.add_argument("--expected-position", type=int,
                        help="Required for motion; refuse if the saved test reference has changed")
    args = parser.parse_args()
    if not -5000 <= args.steps <= 5000:
        parser.error("steps must be within -5000..5000")
    if args.steps and args.expected_position is None:
        parser.error("motion requires --expected-position to guard the calibration reference")
    matches = [p for p in list_ports.comports() if p.device.upper() == args.port.upper()]
    if (len(matches) != 1 or matches[0].serial_number != IDENTITY or
            matches[0].vid != 0x16C0 or matches[0].pid != 0x0483):
        raise RuntimeError(f"Expected steering USB serial {IDENTITY} on {args.port}")
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d-%H%M%SZ")
    runtime = Path(__file__).resolve().parents[2] / "runtime/steering"
    runtime.mkdir(parents=True, exist_ok=True)
    destination = runtime / f"limit-test-{stamp}.json"
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
                               s["event"] == "USB_STOP", 2)
            if not args.steps:
                deadline = time.monotonic() + 2
                while time.monotonic() < deadline:
                    read()
                result["outcome"] = "STATIONARY_ONLY"
            else:
                if initial["pos"] != args.expected_position:
                    raise RuntimeError("Test position changed; refusing to move with an invalid reference")
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
