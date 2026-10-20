"""USB support for the PowerShell CAN-test installer. Never sends actuator commands."""
import argparse
import json
from pathlib import Path
import re
import sys
import time

import serial
from serial.tools import list_ports


def ports():
    return [
        {'port': p.device, 'serial': p.serial_number, 'description': p.description}
        for p in list_ports.comports()
        if p.vid == 0x16C0 and p.pid == 0x0483
    ]


def require_board(port, identity):
    match = next((p for p in ports() if p['port'].upper() == port.upper()), None)
    if not match or not identity or match['serial'] != identity:
        raise RuntimeError(f'USB identity changed: expected {identity} on {port}; refusing access')
    return match


def expected_role(line):
    match = re.match(r'^CAN_TEST_V1 role=(MASTER|THROTTLE|STEERING|BRAKE)\s', line)
    return match.group(1) if match else None


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('list')
    for name in ['reboot', 'verify', 'monitor']:
        cmd = commands.add_parser(name)
        cmd.add_argument('--port', required=True)
        cmd.add_argument('--serial', required=True)
        if name == 'verify':
            cmd.add_argument('--role', choices=['MASTER', 'THROTTLE', 'STEERING', 'BRAKE'], required=True)
            cmd.add_argument('--seconds', type=float, default=8)
        if name == 'monitor':
            cmd.add_argument('--log', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.command == 'list':
        print(json.dumps(ports()))
        return 0
    require_board(args.port, args.serial)
    if args.command == 'reboot':
        # Teensy core usb.c treats 134-baud line coding as a bootloader request.
        # It targets this COM device only; do not broadcast teensy_reboot -s.
        with serial.Serial(args.port, baudrate=134, timeout=0.2):
            pass
        return 0
    if args.command == 'verify':
        deadline = time.monotonic() + args.seconds
        last_line = ''
        with serial.Serial(args.port, baudrate=115200, timeout=0.3) as connection:
            while time.monotonic() < deadline:
                line = connection.readline().decode('ascii', errors='replace').strip()
                if not line:
                    continue
                last_line = line
                role = expected_role(line)
                if role:
                    if role != args.role:
                        raise RuntimeError(f'Wrong firmware role: expected {args.role}, received {role}')
                    print(json.dumps({'port': args.port, 'serial': args.serial,
                                      'role': role, 'sample': line}))
                    return 0
        raise RuntimeError(f'No {args.role} CAN_TEST_V1 output on {args.port}. Last output: {last_line!r}')
    # Monitor is read-only. Store exact raw lines and format each peer on a new line.
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with serial.Serial(args.port, baudrate=115200, timeout=0.5) as connection, \
            args.log.open('a', encoding='utf-8') as log:
        print(f'Reading master {args.port}, serial {args.serial}; Ctrl+C stops.\n', flush=True)
        while True:
            line = connection.readline().decode('ascii', errors='replace').strip()
            if not line:
                continue
            log.write(line + '\n'); log.flush()
            role = expected_role(line)
            if role and role != 'MASTER':
                raise RuntimeError(f'This board reports {role}, not MASTER; monitoring stopped')
            print(line.replace(' | ', '\n  ') + '\n', flush=True)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print('\nMonitor stopped; serial port closed.')
    except (RuntimeError, serial.SerialException, OSError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1)
