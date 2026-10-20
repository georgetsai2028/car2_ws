"""Read-only master/CAN check. Sends no control or actuator commands."""
import argparse
import json
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    parser.add_argument("--seconds", type=float, default=5)
    args = parser.parse_args()
    latest = None
    count = 0
    with serial.Serial(args.port, 115200, timeout=0.3) as device:
        deadline = time.monotonic() + max(1, args.seconds)
        while time.monotonic() < deadline:
            line = device.readline().decode("ascii", errors="ignore").strip()
            try:
                frame = json.loads(line)
            except ValueError:
                continue
            if isinstance(frame, dict) and "e" in frame:
                latest = frame
                count += 1
    print(f"Master telemetry frames: {count}. No commands sent.")
    if latest is None:
        print("No master telemetry. Check the USB port and master firmware.")
        return 2
    print(json.dumps(latest, indent=2))
    for name, key in (("Throttle", "tr"), ("Steering", "sr"), ("Brake", "br")):
        state = "FRESH" if latest.get(key) == 1 else "MISSING / STALE"
        print(f"{name}: {state}")
    if not all(latest.get(key) == 1 for key in ("tr", "sr", "br")):
        print("Check node logic power, CAN transceivers, wiring, and termination.")
        return 2
    print("Steering reference:", "SET" if latest.get("sh") == 1 else "REQUIRED (Z)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
