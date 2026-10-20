"""Probe or run the temporary, finite USB-only acceleration diagnostic."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
import time
import serial

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'runtime/throttle'
sys.path.insert(0, str(ROOT / 'outputs/car2-four-node-can-test'))
from can_test_usb import require_board

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM9')
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--wheels-raised-confirmed', action='store_true')
    parser.add_argument('--vehicle-power-on-confirmed', action='store_true')
    args = parser.parse_args()
    if args.run and not (args.wheels_raised_confirmed and args.vehicle_power_on_confirmed):
        parser.error('Powered test requires raised-wheel and vehicle-power confirmations')
    require_board(args.port, '20543040')
    OUT.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%SZ')
    logpath = OUT / (('run-' if args.run else 'probe-') + stamp + '.jsonl')
    result = {'startedUtc': datetime.now(timezone.utc).isoformat(), 'port': args.port,
              'serial': '20543040', 'log': logpath.name, 'poweredRun': args.run,
              'result': 'INCOMPLETE', 'physicalWheelMotion': 'UNVERIFIED',
              'voltageMeasurement': 'NONE: DAC register counts are not measured analog volts'}
    frames = []
    connection = None
    with logpath.open('w', encoding='utf-8') as log:
        def record(event, **kw):
            log.write(json.dumps({'time': time.monotonic(), 'event': event, **kw}) + '\n')
            log.flush()
        def send(command):
            connection.write(command)
            record('tx', command=command.decode('ascii'))
        def read_frame():
            line = connection.readline().decode('ascii', errors='replace').strip()
            if not line:
                return None
            record('rx', line=line)
            try:
                value = json.loads(line)
            except ValueError:
                return None
            if value.get('fw') == 'THROTTLE_USB_TEST_V1':
                frames.append(value)
                return value
            if value.get('event') in ('bus', 'i2c_probe', 'dac_read'):
                print(json.dumps(value), flush=True)
            return None
        try:
            connection = serial.Serial(args.port, 115200, timeout=0.025, write_timeout=0.2)
            connection.reset_input_buffer()
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and not frames:
                read_frame()
            if not frames:
                raise RuntimeError('USB diagnostic firmware marker not found; no commands sent')
            if frames[-1]['running']:
                send(b'x')
                raise RuntimeError('An unexpected active test was stopped')
            send(b'P')
            deadline = time.monotonic() + 2
            baseline = len(frames)
            while time.monotonic() < deadline:
                read_frame()
                if len(frames) - baseline >= 5:
                    break
            if len(frames) == baseline:
                raise RuntimeError('No post-probe telemetry')
            result['preflight'] = frames[-1]
            print('PREFLIGHT=' + json.dumps(frames[-1]), flush=True)
            if not args.run:
                result['result'] = 'READ_ONLY_PROBE_COMPLETE'
                return
            if not frames[-1]['ready'] or frames[-1]['fault']:
                raise RuntimeError('DAC preflight refused movement: ' + frames[-1]['reason'])
            start = time.monotonic()
            last_rx = start
            next_keepalive = start
            phase_seen = set()
            run_seen = False
            send(b'R')
            while time.monotonic() - start < 9:
                now = time.monotonic()
                if now >= next_keepalive:
                    send(b'K')
                    next_keepalive = now + 0.05
                frame = read_frame()
                if frame:
                    last_rx = time.monotonic()
                    if frame['fault'] or frame['requested'] > 1768:
                        raise RuntimeError('Firmware test fault: ' + json.dumps(frame))
                    if frame['running']:
                        run_seen = True
                        if frame['requested'] != frame['readCount'] or frame['pd'] != 0:
                            raise RuntimeError('DAC readback mismatch')
                        if frame['phase'] not in phase_seen:
                            phase_seen.add(frame['phase'])
                            print('PHASE=' + json.dumps(frame), flush=True)
                    elif run_seen:
                        if frame['reason'] != 'COMPLETE':
                            raise RuntimeError('Test stopped: ' + frame['reason'])
                        if phase_seen != set(range(8)):
                            raise RuntimeError('Not all test phases were observed')
                        result['completedPhases'] = sorted(phase_seen)
                        result['result'] = 'DAC_REGISTERS_FOLLOWED_FINITE_FORWARD_SEQUENCE'
                        break
                if time.monotonic() - last_rx > 0.3:
                    raise RuntimeError('Telemetry timeout')
            else:
                raise RuntimeError('Finite test did not finish in time')
        except Exception as error:
            result['result'] = 'ABORTED'
            result['error'] = repr(error)
            print('ERROR=' + repr(error), flush=True)
        finally:
            if connection is not None:
                try:
                    # Only command verified diagnostic firmware, never unknown application USB.
                    if frames and args.run:
                        send(b'x')
                        deadline = time.monotonic() + 0.6
                        while time.monotonic() < deadline:
                            read_frame()
                        result['final'] = frames[-1]
                        result['neutralConfirmed'] = not frames[-1]['running'] and frames[-1]['gear'] == 'N'
                        result['idleRegisterConfirmed'] = frames[-1]['readCount'] == 993 and frames[-1]['pd'] == 0
                except Exception as error:
                    result['stopError'] = repr(error)
                finally:
                    connection.close()
            if frames:
                result['encoderCountRange'] = [min(f['encoder'] for f in frames), max(f['encoder'] for f in frames)]
                result['maximumAbsoluteEncoderRpm'] = max(abs(f['rpm']) for f in frames)
            result['finishedUtc'] = datetime.now(timezone.utc).isoformat()
            (OUT / ('latest-run.json' if args.run else 'latest-probe.json')).write_text(json.dumps(result, indent=2) + '\n')
            print('RESULT=' + json.dumps(result), flush=True)
            print('SERIAL_CLOSED', flush=True)
    if result['result'] == 'ABORTED':
        raise SystemExit(1)

if __name__ == '__main__':
    main()
