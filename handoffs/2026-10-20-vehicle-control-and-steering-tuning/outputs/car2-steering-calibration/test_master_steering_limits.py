"""Finite master-to-steering test. Never selects a drive gear or throttle."""
from pathlib import Path
from datetime import datetime, timezone
import json
import sys
import time
import serial
import argparse
import hashlib

BASE = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', help='master COM port; omitted detects USB serial 18637540')
parser.add_argument('--center-confirmed', action='store_true',
                    help='Operator has confirmed wheels physically straight and stationary for this test')
args = parser.parse_args()
if not args.center_confirmed:
    parser.error('A current physical straight/stationary confirmation is required')
manifest = json.loads((OUT / 'steering-operational-limits-r01.json').read_text(encoding='utf-8-sig'))
if manifest['firmwareMarker'] != 'STEER_LIMITS_20261019_R01' or not manifest['uploaded']:
    raise RuntimeError('Operational steering image has not been verified installed')
if hashlib.sha256((OUT / manifest['file']).read_bytes()).hexdigest() != manifest['sha256']:
    raise RuntimeError('Packaged steering image hash changed')
if manifest['softLimitLeftPulses'] != -8250 or manifest['softLimitRightPulses'] != 10000:
    raise RuntimeError('Selected steering limits do not match the installed package')
sys.path.insert(0, str(BASE / 'outputs/car2-four-node-can-test'))
from can_test_usb import require_board, ports

SAFE = b'A E=1 T=0 M=N B=1 S=0\n'
if not args.port:
    matches = [item for item in ports() if item['serial'] == '18637540']
    if len(matches) != 1:
        raise RuntimeError('Connect master USB serial 18637540; its COM port will be detected automatically')
    args.port = matches[0]['port']
require_board(args.port, '18637540')
RUN = BASE / 'runtime/master'
RUN.mkdir(parents=True, exist_ok=True)
log_path = RUN / ('steering-master-test-' + datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ') + '.jsonl')
summary = {'startedUtc': datetime.now(timezone.utc).isoformat(),
           'operatorConfirmedStraightAndStationary': args.center_confirmed,
           'usbSerial': '18637540', 'port': args.port, 'log': log_path.name,
           'steeringImageSha256': manifest['sha256'],
           'steeringLimitsPulses': [-8250, 10000],
           'commandedThrottle': 0, 'commandedGear': 'N', 'commandedBrake': 1,
           'phases': [], 'result': 'NOT_COMPLETED', 'physicalMovement': 'AWAITING_OPERATOR_OBSERVATION'}
latest = None
latest_at = 0.0
connection = None

with log_path.open('w', encoding='utf-8') as log:
    def note(event, **fields):
        log.write(json.dumps({'time': time.monotonic(), 'event': event, **fields}) + '\n')
        log.flush()

    def send(command):
        connection.write(command)
        note('tx', command=command.decode('ascii').strip())

    def read_frame():
        global latest, latest_at
        line = connection.readline().decode('ascii', errors='replace').strip()
        if not line:
            return None
        note('rx', line=line)
        try:
            frame = json.loads(line)
        except ValueError:
            return None
        if isinstance(frame, dict) and all(k in frame for k in ('e', 't', 'm', 'b', 's', 'tr', 'sr', 'br', 'sh', 'sf', 'sp')):
            latest = frame
            latest_at = time.monotonic()
            return frame
        return None

    def check_common(frame):
        if not all(frame[k] == 1 for k in ('tr', 'sr', 'br')):
            raise RuntimeError('Lost normal node feedback: ' + json.dumps(frame))
        if frame['t'] != 0 or frame['m'] != 'N' or frame['b'] != 1:
            raise RuntimeError('Unexpected throttle, gear or brake command state')
        if frame['sf'] & 0x13:  # either switch or both-switch fault
            raise RuntimeError('Steering limit/fault asserted')

    def phase(name, seconds, command, active=False, require_reference=False):
        start = time.monotonic()
        next_send = start
        frames = []
        print('PHASE ' + name, flush=True)
        while time.monotonic() - start < seconds:
            now = time.monotonic()
            if now >= next_send:
                send(command)
                next_send = now + 0.05
            frame = read_frame()
            if frame:
                check_common(frame)
                if require_reference and frame['sh'] != 1:
                    raise RuntimeError('Steering reference lost')
                if active and time.monotonic() - start > 0.4:
                    if frame['e'] != 0 or frame['sf'] & 0x9C:
                        raise RuntimeError('Steering active command inhibited: ' + json.dumps(frame))
                frames.append(frame)
            if now - latest_at > 0.4 and now - start > 0.4:
                raise RuntimeError('Master telemetry timeout')
        if len(frames) < 5:
            raise RuntimeError('Insufficient fresh telemetry in ' + name)
        item = {'name': name, 'samples': len(frames), 'spMin': min(f['sp'] for f in frames),
                'spMax': max(f['sp'] for f in frames), 'last': frames[-1]}
        summary['phases'].append(item)
        print('FEEDBACK ' + json.dumps(item), flush=True)
        return frames

    try:
        connection = serial.Serial(args.port, 115200, timeout=0.025, write_timeout=0.2)
        connection.reset_input_buffer()
        phase('E-stop baseline', 0.8, SAFE)
        send(b'Z\n')  # operator-confirmed physical straight position; no step command
        reference = phase('Record stationary center', 0.8, SAFE)
        if reference[-1]['sh'] != 1 or reference[-1]['sp'] != 0 or reference[-1]['e'] != 1:
            raise RuntimeError('Stationary center reference did not acknowledge')
        print('CENTER_REFERENCE_CONFIRMED', flush=True)
        for name, steer, expected in [('RIGHT_SOFT_BOUNDARY', 1.0, 1), ('CENTER', 0.0, 0),
                                      ('LEFT_SOFT_BOUNDARY', -1.0, -1), ('CENTER', 0.0, 0)]:
            command = ('A E=0 T=0 M=N B=1 S=' + str(steer) + '\n').encode('ascii')
            frames = phase(name, 3.5, command, active=True, require_reference=True)
            actual = frames[-1]['sp']
            if expected > 0 and actual < 125:
                raise RuntimeError('Steering software position did not reach right test target')
            if expected < 0 and actual > -125:
                raise RuntimeError('Steering software position did not reach left test target')
            if expected == 0 and abs(actual) > 2:
                raise RuntimeError('Steering software position did not return to center')
        phase('E-stop reference retention', 2.0, SAFE, require_reference=True)
        summary['result'] = 'PASS_SOFTWARE_POSITION_AND_NORMAL_CAN_COMMAND_PATH'
    except Exception as error:
        summary['result'] = 'ABORTED'
        summary['error'] = repr(error)
        print('ABORT=' + repr(error), flush=True)
    finally:
        if connection is not None and connection.is_open:
            try:
                send(SAFE)
                deadline = time.monotonic() + 0.8
                next_send = time.monotonic() + 0.05
                while time.monotonic() < deadline:
                    if time.monotonic() >= next_send:
                        send(SAFE)
                        next_send = time.monotonic() + 0.05
                    read_frame()
                summary['finalTelemetry'] = latest
                summary['finalEstopAcknowledged'] = bool(latest and time.monotonic()-latest_at < 0.4 and latest['e'] == 1 and latest['t'] == 0 and latest['m'] == 'N')
                summary['finalReferenceRetained'] = bool(latest and latest['sh'] == 1)
            except Exception as error:
                summary['stopError'] = repr(error)
            finally:
                connection.close()
        summary['finishedUtc'] = datetime.now(timezone.utc).isoformat()
        (RUN / 'steering-master-test-status.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        print('RESULT=' + json.dumps(summary), flush=True)
        print('SERIAL_CLOSED', flush=True)

if summary['result'].startswith('ABORTED') or not summary.get('finalEstopAcknowledged') or not summary.get('finalReferenceRetained'):
    raise SystemExit(1)
