"""Passive steering USB check. No commands, resets, uploads or motor movement."""
from datetime import datetime, timezone
import argparse
import json
from pathlib import Path
import re
import sys
import time
import serial

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'outputs/car2-four-node-can-test'))
from can_test_usb import require_board
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', default='COM5')
parser.add_argument('--seconds', type=float, default=6)
args = parser.parse_args()
require_board(args.port, '20369970')
rows, raw, errors = [], [], []
link = None
try:
    link = serial.Serial(args.port, 115200, timeout=0.2)
    deadline = time.monotonic() + min(max(args.seconds, 1), 15)
    while time.monotonic() < deadline:
        line = link.readline().decode('ascii', errors='replace').strip()
        if not line:
            continue
        raw.append(line)
        match = re.search(r'R_limit=(PRESSED|open)\s+L_limit=(PRESSED|open)\s+FAULT=(YES|no)\s+pos=(-?\d+)\s+enc=(\d+)', line)
        if match:
            rows.append(dict(right=match[1], left=match[2], fault=match[3],
                             position=int(match[4]), encoderRaw=int(match[5])))
except (serial.SerialException, OSError) as error:
    errors.append(repr(error))
finally:
    if link is not None:
        link.close()
out = root / 'runtime/steering'
out.mkdir(parents=True, exist_ok=True)
stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ')
log = out / ('stationary-' + stamp + '.log')
log.write_text('\n'.join(raw) + ('\n' if raw else ''), encoding='utf-8')
result = dict(observedUtc=datetime.now(timezone.utc).isoformat(),
              port=args.port, usbSerial='20369970', statusFrames=len(rows),
              first=rows[0] if rows else None, last=rows[-1] if rows else None,
              encoderRange=[min(r['encoderRaw'] for r in rows), max(r['encoderRaw'] for r in rows)] if rows else None,
              rightStates=sorted(set(r['right'] for r in rows)),
              leftStates=sorted(set(r['left'] for r in rows)),
              faultStates=sorted(set(r['fault'] for r in rows)),
              commandsSent=False, firmwareChanged=False, serialPortClosed=True,
              errors=errors, log=log.name,
              limit='Stationary readings do not validate full-range sensor linearity, input voltage or travel calibration.')
(out / ('stationary-' + stamp + '.json')).write_text(json.dumps(result, indent=2) + '\n')
(out / 'latest-stationary-test.json').write_text(json.dumps(result, indent=2) + '\n')
print(json.dumps(result, indent=2))
if not rows:
    print('RAW=' + json.dumps(raw[:8]))
raise SystemExit(2 if errors or not rows else 0)
