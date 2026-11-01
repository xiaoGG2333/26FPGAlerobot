"""No-motion telemetry and optional inactive Zynq IK probes; no torque/EEPROM writes."""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import re

from right_joycon import UART, Mapper, NAMES, fk_pose

ROOT = Path(__file__).resolve().parents[1]


def measured_q(counts, mapping):
    joints = [mapping['joint_mapping'][name] for name in mapping['joint_order']]
    return [j['sign']*(count-j['zero_count'])/j['counts_per_rad']
            for count, j in zip(counts, joints)]


def pose_readiness(counts, mapping):
    q = measured_q(counts, mapping)
    joints = [mapping['joint_mapping'][name] for name in mapping['joint_order']]
    issues = []
    for name, count, angle, joint in zip(mapping['joint_order'], counts, q, joints):
        if not joint['range_min']+10 <= count <= joint['range_max']-10:
            issues.append({'id': joint['id'], 'joint': name, 'reason': 'ENCODER_LIMIT_MARGIN', 'count': count})
        if not joint['q_min_rad'] <= angle <= joint['q_max_rad']:
            issues.append({'id': joint['id'], 'joint': name, 'reason': 'MODEL_ANGLE_LIMIT',
                           'count': count, 'angle_deg': math.degrees(angle),
                           'allowed_deg': [math.degrees(joint['q_min_rad']), math.degrees(joint['q_max_rad'])]})
    if abs(fk_pose(q)[4]) > 1.4:
        issues.append({'reason': 'POSE1_PITCH_LIMIT'})
    return {'pose_in_model_limits': not issues, 'issues': issues,
            'physical_alignment_verified': mapping['physical_alignment_verified']}


def analyze_csv(path, mapping):
    rows = list(csv.DictReader(path.open(encoding='utf-8')))
    moving = [r for r in rows if r['status'] == 'CONVERGED'
              and all(r.get('actual_count_id%d' % i) for i in range(1, 6))]
    report = {'file': str(path), 'moving_frames': len(moving),
              'coordinates': 'model FK from encoders; physical calibration is not verified'}
    if not moving:
        return report
    report['max_follow_error_counts'] = {
        name: max(abs(int(r['count_id%d' % (i+1)])-int(r['actual_count_id%d' % (i+1)]))
                  for r in moving) for i, name in enumerate(NAMES)}
    runs, current = [], []
    for r in rows:
        # Stick mask bit 4 is R = Z+. Split on every other input or hold.
        if r in moving and int(r['button']) == 16:
            current.append(r)
        else:
            if len(current) >= 2:
                runs.append(current)
            current = []
    if len(current) >= 2:
        runs.append(current)
    report['r_only_runs'] = []
    for run in runs:
        first, last = run[0], run[-1]
        actual = [fk_pose(measured_q([int(r['actual_count_id%d' % i])
                                    for i in range(1, 6)], mapping)) for r in (first, last)]
        report['r_only_runs'].append({
            'start_s': float(first['elapsed_s']), 'end_s': float(last['elapsed_s']),
            'requested_dz_mm': 1000*(float(last['target_z'])-float(first['target_z'])),
            'encoder_model_dz_mm': 1000*(actual[1][2]-actual[0][2]),
            'first_counts': [int(first['actual_count_id%d' % i]) for i in range(1, 6)],
            'last_counts': [int(last['actual_count_id%d' % i]) for i in range(1, 6)]})
    return report


def servo_read(uart, motor):
    command = 'SERVO1 READ %d\n' % motor
    line = uart.exchange(command.encode('ascii'), 'SERVO1 ID=%d OK' % motor)
    # The next exchange skips the trailing SERVO1 END; do not change bus baud.
    values = dict(re.findall(r'(\w+)=([^\s]+)', line))
    return {key: float(value) if key == 'voltage' else int(value)
            for key, value in values.items()}


def live_report(port, timeout, probe, mapping):
    uart = UART(port, timeout)
    report = {'port': port, 'torque_enable_sent': False, 'eeprom_write_sent': False,
              'motion_commands_sent': False, 'ik_probes': []}
    try:
        report['hello'] = uart.exchange(b'HELLO\n', 'POSE1 READY ')
        report['arm_state'] = uart.arm('ARM1 STATE', 'STATE')
        if not re.search(r'\bactive=0\b', report['arm_state']):
            raise RuntimeError('Arm is active. End the running control session before stationary diagnostics.')
        report['servos'] = [servo_read(uart, i) for i in range(1, 7)]
        counts = [s['raw_pos'] for s in report['servos'][:5]]
        q = measured_q(counts, mapping)
        report['q_deg'] = [math.degrees(v) for v in q]
        report['encoder_model_pose'] = fk_pose(q)
        report['pose_readiness'] = pose_readiness(counts, mapping)
        report['healthy_stationary'] = all(s['torque'] == 0 and s['moving'] == 0
            and 9 <= s['voltage'] <= 12.6 and s['temp'] <= 55 for s in report['servos'])
        if probe:
            if not report['healthy_stationary']:
                raise RuntimeError('IK probes require all six axes stationary, torque off and healthy.')
            if not report['pose_readiness']['pose_in_model_limits']:
                raise RuntimeError('Measured pose is outside model/protocol limits: '+json.dumps(report['pose_readiness']['issues']))
            check = uart.arm('ARM1 TEST_MANUAL_CHECK', 'CHECK_OK')
            if not check.startswith('ARM1 CHECK_OK '):
                raise RuntimeError('Read-only preflight rejected: '+check)
            q, pose = uart.state()
            if abs(pose[4]) > 1.4:
                raise RuntimeError('Current pitch is outside POSE1 +/-1.4 rad; no probes sent.')
            config = json.loads((ROOT/'host/right_config.json').read_text())
            config['workspace_min'] = [-.5, -.6, -.3]
            config['workspace_max'] = [.6, .6, .7]
            for axis in range(3):
                for sign in (-1, 1):
                    # Re-seed every independent probe from the measured encoders.
                    check = uart.arm('ARM1 TEST_MANUAL_CHECK', 'CHECK_OK')
                    if not check.startswith('ARM1 CHECK_OK '):
                        raise RuntimeError('Probe preflight rejected: '+check)
                    delta = [0.]*6
                    delta[axis] = sign*.002
                    target = Mapper(config, pose, [0.]*6).target(delta, 1.)
                    result = uart.solve(0, target, .5)
                    result['axis'] = 'XYZ'[axis]
                    result['sign'] = sign
                    result['target'] = target
                    result['delta_q_deg'] = [math.degrees(b-a) for a, b in zip(q, result['q'])]
                    result['solved_fk'] = fk_pose(result['q'])
                    report['ik_probes'].append(result)
        report['status'] = 'PASS'
    except (OSError, ValueError, RuntimeError, TimeoutError) as error:
        report['status'] = 'FAIL'
        report['error'] = str(error)
    finally:
        uart.close()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--timeout', type=float, default=5.)
    parser.add_argument('--probe-ik', action='store_true', help='Solve +/-2 mm targets while arm remains inactive')
    parser.add_argument('--csv', type=Path, nargs='*', default=[])
    parser.add_argument('--out', type=Path, default=ROOT/'results/teleop_diagnosis.json')
    args = parser.parse_args()
    if not args.port and not args.csv:
        parser.error('Supply --port or --csv')
    if args.probe_ik and not args.port:
        parser.error('--probe-ik requires --port')
    mapping = json.loads((ROOT/'host/right_mapping.json').read_text())
    report = {'date_utc': datetime.now(timezone.utc).isoformat(),
              'physical_alignment_verified': mapping['physical_alignment_verified'],
              'logs': [analyze_csv(p, mapping) for p in args.csv]}
    if args.port:
        try:
            report['board'] = live_report(args.port, args.timeout, args.probe_ik, mapping)
        except (OSError, ValueError) as error:
            report['board'] = {'status': 'FAIL', 'error': str(error)}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if report.get('board', {}).get('status') == 'FAIL':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
