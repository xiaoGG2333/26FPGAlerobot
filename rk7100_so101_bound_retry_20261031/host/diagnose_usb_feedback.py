"""READ-only temperature cross-check, varying read ranges and request spacing.

No WRITE, torque enable, mode change, or automatic calibration is performed.
Temperature below the exit threshold does not prove that movement is safe.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from auto_calibrate_usb import USBProtocol
from auto_calibrate_zynq import CalibrationError, signed, word

PATTERNS = ((63, 1), (62, 2), (56, 15), (60, 11), (48, 32))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--motor', type=int, default=5, choices=range(1, 7))
    parser.add_argument('--rounds', type=int, default=50)
    parser.add_argument('--pause', type=float, default=.05)
    parser.add_argument('--isolated', action='store_true',
                        help='Check only --motor when it is the only servo connected to the adapter')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    if not 1 <= args.rounds <= 100 or not .02 <= args.pause <= .1:
        parser.error('rounds 1..100 and pause .02.. .1 required')
    folder = args.out or Path(__file__).resolve().parents[1] / 'results' / (
        'usb_read_only_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    if folder.exists():
        parser.error('Preserve the existing output directory')
    folder.mkdir(parents=True)
    report = {'status': 'NOT_COMPLETED', 'read_only': True,
              'motion_commands_sent': False, 'port': args.port, 'motor': args.motor,
              'rounds_per_phase': args.rounds, 'pause_s': args.pause,
              'checked_motors': [args.motor] if args.isolated else list(range(1, 7)),
              'before': [], 'samples': [], 'real_calibration_complete': False,
              'proves_movement_reliability': False}
    bus = None
    before_eeprom = {}
    deadline = time.monotonic() + 90

    def health(data, motor):
        if not 90 <= data[6] <= 126 or data[7] > 55 or data[9]:
            raise CalibrationError(f'ID{motor}: unhealthy stationary feedback')

    try:
        bus = USBProtocol(args.port, folder)
        for motor in report['checked_motors']:
            eeprom = bus.read(motor, 0, 32) + bus.read(motor, 32, 8)
            torque = bus.read(motor, 40, 1)[0]
            data = bus.read(motor, 56, 15)
            item = {'id': motor, 'torque': torque, 'mode': eeprom[33],
                    'eeprom_hex': eeprom.hex(), 'temperature': data[7],
                    'voltage': data[6] / 10, 'position': word(data[:2])}
            report['before'].append(item)
            before_eeprom[motor] = eeprom
            if word(eeprom[3:5]) != 777 or eeprom[5] != motor or torque or eeprom[33]:
                raise CalibrationError('Requires the checked STS3215 axes in Torque0/Mode0')
            health(data, motor)
        print('READ ONLY: checked motors ' + str(report['checked_motors']) + ' Torque0/Mode0 confirmed', flush=True)
        for phase, pause in (('fast', 0), ('spaced', args.pause)):
            for turn in range(args.rounds):
                if bus.log_error:
                    raise CalibrationError('USB diagnostic packet log failed: ' + bus.log_error)
                if (folder / 'STOP').exists() or time.monotonic() >= deadline:
                    raise CalibrationError('READ-only STOP/deadline reached')
                if bus.read(args.motor, 40, 1) != b'\0' or bus.read(args.motor, 33, 1) != b'\0':
                    raise CalibrationError('Mode or torque changed outside this diagnostic')
                # Rotate the order, so a pattern is not always the first read.
                patterns = PATTERNS[turn % len(PATTERNS):] + PATTERNS[:turn % len(PATTERNS)]
                for address, count in patterns:
                    if pause:
                        time.sleep(pause)
                    data = bus.read(args.motor, address, count)
                    row = {'phase': phase, 'round': turn, 'time': time.time(),
                           'address': address, 'count': count,
                           'temperature': data[63 - address], 'data_hex': data.hex()}
                    report['samples'].append(row)
                    if address <= 62:
                        row['voltage'] = data[62 - address] / 10
                        if not 9 <= row['voltage'] <= 12.6:
                            raise CalibrationError('Stationary voltage outside bounds')
                    if address + count > 65:
                        row['status'] = data[65 - address]
                        if row['status']:
                            raise CalibrationError('Stationary fault status')
                    if address <= 56:
                        row['position'] = signed(data[56 - address:58 - address], 15)
                        row['speed'] = signed(data[58 - address:60 - address], 15)
                        row['load'] = signed(data[60 - address:62 - address], 10)
                    if row['temperature'] > 55:
                        raise CalibrationError('Stationary temperature exceeds 55; no motion issued')
            phase_rows = [x for x in report['samples'] if x['phase'] == phase]
            print(phase + ' temp=' + str([min(x['temperature'] for x in phase_rows),
                                         max(x['temperature'] for x in phase_rows)]), flush=True)
        report['phase_statistics'] = {}
        for phase in ('fast', 'spaced'):
            report['phase_statistics'][phase] = {}
            for address, count in PATTERNS:
                temperatures = [x['temperature'] for x in report['samples']
                                if x['phase'] == phase and x['address'] == address and x['count'] == count]
                report['phase_statistics'][phase][f'{address}/{count}'] = {
                    'samples': len(temperatures), 'min': min(temperatures), 'max': max(temperatures),
                    'values': {str(value): temperatures.count(value) for value in sorted(set(temperatures))}}
        report['after'] = []
        for motor in report['checked_motors']:
            eeprom = bus.read(motor, 0, 32) + bus.read(motor, 32, 8)
            torque = bus.read(motor, 40, 1)[0]
            data = bus.read(motor, 56, 15)
            health(data, motor)
            report['after'].append({'id': motor, 'torque': torque, 'mode': eeprom[33],
                                    'eeprom_matches_before': eeprom == before_eeprom[motor],
                                    'temperature': data[7]})
            if torque or eeprom[33] or not report['after'][-1]['eeprom_matches_before']:
                raise CalibrationError('Final torque/mode/EEPROM changed outside diagnostic')
        report['status'] = 'READ_ONLY_CHECK_COMPLETE'
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        try:
            if bus is not None:
                report['packet_log_error'] = bus.log_error
                bus.close()
        finally:
            (folder / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf8')
            print('Recorded ' + str(folder), flush=True)


if __name__ == '__main__':
    try:
        main()
    except (CalibrationError, OSError, ValueError, KeyboardInterrupt) as error:
        raise SystemExit(str(error))
