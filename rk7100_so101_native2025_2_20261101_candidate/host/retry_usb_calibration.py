"""Bounded NEW sessions after an unconfirmed high followed by verified STOP.

No reading is deleted/substituted. Any reproduced high, device alarm, operator
STOP, transport/log/rollback error aborts. Never resume partial endpoint state.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import auto_calibrate_usb as usb
from auto_calibrate_zynq import CalibrationError


def run_attempts(folder, port='COM12', velocity=200, torque=400, attempts=3,
                 execute=usb.main):
    report = {'status': 'NOT_COMPLETED', 'port': port, 'attempts': [],
              'real_calibration_complete': False, 'commit_confirmed': False,
              'initial_high_readings_proven_false': False,
              'physical_alignment_verified': False, 'geometry_verified': False,
              'maximum_sessions': attempts, 'old_endpoint_reuse': False}
    stop = folder / 'STOP'
    seen_high_axes = set()
    try:
        for index in range(1, attempts + 1):
            if stop.exists():
                raise CalibrationError('Operator STOP; no restart')
            target = folder / f'attempt_{index:02d}'
            entry = {'folder': target.name}
            report['attempts'].append(entry)
            print(f'FULL CALIBRATION SESSION {index}/{attempts}: {target}', flush=True)
            try:
                execute(['--port', port, '--out', str(target), '--velocity', str(velocity),
                         '--torque', str(torque), '--stop-file', str(stop)])
            except usb.TemperatureNotReproduced as error:
                entry['status'] = 'STOPPED_HIGH_NOT_REPRODUCED'
                entry['error'] = str(error)
                child = json.loads((target / 'summary.json').read_text(encoding='utf8'))
                check = child.get('temperature_recheck') or {}
                if (stop.exists() or child.get('transport_log_error') or child.get('cleanup_error') or child.get('cleanup_output_error') or child.get('close_error')
                        or check.get('classification') != 'HIGH_NOT_REPRODUCED_AFTER_CONFIRMED_STOP'
                        or not check.get('cleanup_confirmed') or len(check.get('samples', [])) != 5
                        or check.get('recheck_duration_s', 0) < 2
                        or child.get('commit_confirmed')
                        or 'off_confirmed=1 rollback_success=1' not in ' '.join(child.get('cleanup', []))):
                    raise CalibrationError('New-session preconditions not verified')
                axis = check.get('id')
                if axis not in range(1, 7) or axis in seen_high_axes:
                    raise CalibrationError('Repeated high on the same axis; no further restart')
                seen_high_axes.add(axis)
                report['high_feedback_axes'] = sorted(seen_high_axes)
                if index == attempts:
                    raise CalibrationError('Repeated temperature anomaly; bounded retry limit reached')
                print('Initial high remains unproven; old endpoints discarded. Waiting OFF before a NEW session.', flush=True)
                end = time.monotonic() + 5
                while time.monotonic() < end:
                    if stop.exists():
                        raise CalibrationError('Operator STOP during OFF cooldown; no restart')
                    time.sleep(min(.04, end - time.monotonic()))
                continue
            child = json.loads((target / 'summary.json').read_text(encoding='utf8'))
            if (child.get('status') != 'CALIBRATION_PASS'
                    or not child.get('real_calibration_complete') or not child.get('commit_confirmed')
                    or child.get('transport_log_error') or child.get('cleanup_output_error') or child.get('close_error')):
                raise CalibrationError('Child did not confirm complete calibration and COMMIT')
            entry['status'] = 'CALIBRATION_PASS'
            report.update(status='CALIBRATION_PASS', real_calibration_complete=True,
                          commit_confirmed=True, successful_results=target.name)
            print(f'CALIBRATION PASS: complete verified result in {target}', flush=True)
            return report
    except BaseException as error:
        report['error'] = repr(error)
        if report['attempts']:
            # Output failure after EEPROM COMMIT must never be misreported as
            # an uncommitted rollback. Preserve the child's confirmed state.
            try:
                child_path = folder / report['attempts'][-1]['folder'] / 'summary.json'
                child = json.loads(child_path.read_text(encoding='utf8'))
                if child.get('commit_confirmed'):
                    report.update(status='COMMITTED_OUTPUT_ERROR', commit_confirmed=True,
                                  committed_results=report['attempts'][-1]['folder'])
            except Exception as summary_error:
                report['child_summary_read_error'] = repr(summary_error)
        print('CALIBRATION STOPPED: ' + str(error), flush=True)
        raise
    finally:
        (folder / 'summary.json').write_text(json.dumps(report, indent=2), encoding='utf8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='COM12')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--velocity', type=int, default=200)
    parser.add_argument('--torque', type=int, default=400)
    parser.add_argument('--attempts', type=int, default=3)
    args = parser.parse_args(argv)
    if not 1 <= args.attempts <= 3 or not 50 <= args.velocity <= 300 or not 100 <= args.torque <= 400:
        parser.error('Sessions 1..3, velocity 50..300, torque 100..400 required')
    folder = args.out or Path(__file__).resolve().parents[1] / 'results' / ('usb_rechecked_' + datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S'))
    folder.mkdir()  # Preserve every previous attempt and its raw data.
    return run_attempts(folder, args.port, args.velocity, args.torque, args.attempts)


if __name__ == '__main__':
    try:
        main()
    except (CalibrationError, OSError, ValueError, KeyboardInterrupt) as error:
        raise SystemExit(str(error))
