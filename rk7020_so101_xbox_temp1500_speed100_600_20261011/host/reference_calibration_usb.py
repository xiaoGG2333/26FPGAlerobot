"""Read-only USB reference capture using the existing per-arm calibration.

Snapshot does not assign any physical model angles. Capture requires known
angles supplied by the operator. Fit produces an unverified candidate only.
No action writes motor registers or enables motion.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import statistics
import time

from auto_calibrate_usb import USBProtocol, TemperatureDebounce
from auto_calibrate_zynq import CalibrationError, signed, word
import model_calibration as model

ALL_NAMES = model.NAMES + ['gripper']


def load_baseline(path):
    data = json.loads(path.read_text(encoding='utf8'))
    if (data.get('schema') != 'so101_project_mapping_v1'
        or data.get('joint_order') != model.NAMES
        or data.get('position_space') != 'servo_Present_Position_after_existing_homing_offset'
        or data.get('counts_per_rev') != 4096 or not data.get('do_not_add_homing_offset_again')):
        raise ValueError('Unsupported encoder baseline convention')
    for index, name in enumerate(ALL_NAMES):
        j = data['joint_mapping'][name]
        if (j['id'] != index + 1 or (j['range_min'], j['range_max']) != model.LIMITS[index]
            or j['homing_offset_signed'] != model.OFFSETS[index]):
            raise ValueError('Baseline differs from the existing reference fitting configuration')
        if index < 5:
            values = [j[k] for k in ('zero_count', 'counts_per_rad', 'q_min_rad', 'q_max_rad')]
            if (not all(math.isfinite(v) for v in values) or j['sign'] not in (-1, 1)
                or abs(j['counts_per_rad'] - model.CPR) > 1e-6
                or not j['range_min'] <= j['zero_count'] <= j['range_max']
                or j['q_min_rad'] >= j['q_max_rad']):
                raise ValueError('Invalid joint mapping: ' + name)
    return data


def validate_eeprom(motor, raw, baseline):
    j = baseline['joint_mapping'][ALL_NAMES[motor - 1]]
    if (len(raw) != 40 or word(raw[3:5]) != 777 or raw[5] != motor
        or raw[33] != 0 or word(raw[9:11]) != j['range_min']
        or word(raw[11:13]) != j['range_max']
        or signed(raw[31:33], 11) != j['homing_offset_signed']):
        raise CalibrationError(f'ID{motor}: live EEPROM does not match the previous per-arm baseline')


def feedback(bus, motor, baseline, thermal):
    raw = bus.read(motor, 56, 15)
    torque = bus.read(motor, 40, 1)[0]
    j = baseline['joint_mapping'][ALL_NAMES[motor - 1]]
    item = dict(id=motor, raw_pos=signed(raw[:2], 15), speed=signed(raw[2:4], 15),
                load=signed(raw[4:6], 10), voltage=raw[6] / 10, temp=raw[7],
                moving=raw[10], current_raw=word(raw[13:15]), torque=torque, alarm=raw[9])
    if (torque or item['moving'] or item['speed'] or item['alarm']
        or not 9 <= item['voltage'] <= 12.6
        or not j['range_min'] <= item['raw_pos'] <= j['range_max']):
        raise CalibrationError(f'ID{motor}: capture requires stationary Torque0/Mode0 and healthy feedback')
    thermal.observe(motor, item['temp'], 'read-only reference capture READ56/15')
    return item


def preview(baseline, counts):
    if len(counts) != 6 or not all(math.isfinite(float(c)) for c in counts):
        raise ValueError('Six finite present-position values required')
    rows = []
    for i, name in enumerate(model.NAMES):
        j = baseline['joint_mapping'][name]
        q = j['sign'] * (counts[i] - j['zero_count']) / j['counts_per_rad']
        rows.append(dict(name=name, id=i + 1, present_position=counts[i], q_rad_preview=q,
                         q_deg_preview=math.degrees(q),
                         in_eeprom_range=j['range_min'] <= counts[i] <= j['range_max'],
                         in_model_range=j['q_min_rad'] <= q <= j['q_max_rad']))
    return dict(joints=rows, preview_only=True, physical_alignment_verified=False,
                geometry_verified=False, installation_collision_clearance_verified=False,
                automatic_motion_enabled=False,
                angle_source='previous project convention, not a measured physical pose')


def collect(port, folder, baseline, samples=5, interval=.05, bus_factory=USBProtocol):
    if not 3 <= samples <= 30 or not .02 <= interval <= .2:
        raise ValueError('samples 3..30 and interval .02.. .2 required')
    folder.mkdir(parents=True, exist_ok=False)
    record = dict(schema='so101_encoder_snapshot_v1', status='NOT_COMPLETED',
                  captured_utc=datetime.now(timezone.utc).isoformat(), port=port,
                  transport='direct USB 1 Mbaud', model=model.MODEL, model_fingerprint=model.FINGERPRINT,
                  joint_order=model.NAMES, position_space='Present_Position_after_existing_homing_offset',
                  limits=[list(x) for x in model.LIMITS], homing_offsets=model.OFFSETS,
                  physical_alignment_verified=False, automatic_motion_enabled=False,
                  motion_commands_sent=False, register_writes_sent=False, samples=[])
    bus = None
    thermal = TemperatureDebounce()
    original = {}
    deadline = time.monotonic() + 30

    def check():
        if (folder / 'STOP').exists() or time.monotonic() > deadline:
            raise CalibrationError('Read-only STOP/deadline reached')
        thermal.deadlines()
        if getattr(bus, 'log_error', None):
            raise CalibrationError('Packet log failed')

    def poll_pending():
        while thermal.pending:
            for motor in list(thermal.pending):
                check()
                record.setdefault('thermal_rechecks', []).append(feedback(bus, motor, baseline, thermal))
            if thermal.pending:
                time.sleep(.02)

    try:
        bus = bus_factory(port, folder)
        for motor in range(1, 7):
            check()
            raw = bus.read(motor, 0, 32) + bus.read(motor, 32, 8)
            validate_eeprom(motor, raw, baseline)
            original[motor] = raw
            # Check before proceeding, not after collecting all six axes.
            feedback(bus, motor, baseline, thermal)
            poll_pending()
        record['eeprom_before_hex'] = {str(k): v.hex() for k, v in original.items()}
        for turn in range(samples):
            readings = []
            for motor in range(1, 7):
                check()
                readings.append(feedback(bus, motor, baseline, thermal))
                poll_pending()
            record['samples'].append(readings)
            if turn + 1 < samples:
                time.sleep(interval)
        spreads = [max(s[i]['raw_pos'] for s in record['samples']) - min(s[i]['raw_pos'] for s in record['samples']) for i in range(6)]
        if max(spreads) > 5:
            raise CalibrationError('Reference pose moved by more than 5 counts during sampling')
        counts = [statistics.median(s[i]['raw_pos'] for s in record['samples']) for i in range(6)]
        record.update(encoder_counts_median=counts, encoder_counts_spread=spreads,
                      previous_mapping_preview=preview(baseline, counts))
        after = {}
        for motor in range(1, 7):
            check()
            raw = bus.read(motor, 0, 32) + bus.read(motor, 32, 8)
            if raw != original[motor]:
                raise CalibrationError(f'ID{motor}: EEPROM changed during capture')
            after[str(motor)] = raw.hex()
            feedback(bus, motor, baseline, thermal)
            poll_pending()
        record.update(status='SNAPSHOT_PASS', eeprom_after_hex=after,
                      eeprom_unchanged=True, six_axes_torque_off=True, position_mode=True)
        check()
    except BaseException as error:
        record['error'] = repr(error)
        raise
    finally:
        record['temperature_policy'] = thermal.report()
        if bus is not None:
            record['transport_log_error'] = getattr(bus, 'log_error', None)
            try:
                bus.close()
            except BaseException as error:
                record['status'] = 'NOT_COMPLETED'
                record['close_error'] = repr(error)
                raise
            finally:
                (folder / 'summary.json').write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf8')
        else:
            (folder / 'summary.json').write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf8')
    return record


def fit_references(captures):
    if len(captures) < 2:
        raise ValueError('At least two distinct known reference captures required')
    for capture in captures:
        if (capture.get('status') != 'SNAPSHOT_PASS' or not capture.get('eeprom_unchanged')
            or not capture.get('six_axes_torque_off') or not capture.get('position_mode')
            or capture.get('register_writes_sent') is not False):
            raise ValueError('Only complete read-only known-reference captures accepted')
    if len({c.get('baseline_sha256') for c in captures}) != 1 or not captures[0].get('baseline_sha256'):
        raise ValueError('All reference captures must use the same previous baseline')
    return model.fit(captures)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--baseline', type=Path, default=Path(__file__).with_name('right_mapping.json'))
    sub = p.add_subparsers(dest='action', required=True)
    for action in ('snapshot', 'capture'):
        s = sub.add_parser(action)
        s.add_argument('--port', default='COM12')
        s.add_argument('--out', type=Path, required=True, help='New directory for capture and packet log')
        s.add_argument('--samples', type=int, default=5)
        if action == 'capture':
            s.add_argument('--label', required=True)
            s.add_argument('--q-deg', nargs=5, type=float, required=True,
                           help='Known physical model angles, not desired targets or old mapping previews')
    s = sub.add_parser('fit')
    s.add_argument('captures', nargs='+', type=Path, help='capture.json files')
    s.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    baseline = load_baseline(a.baseline)
    if a.out.exists():
        p.error('Preserve existing output; use a new path')
    if a.action == 'fit':
        data = fit_references([json.loads(x.read_text(encoding='utf8')) for x in a.captures])
        a.out.parent.mkdir(parents=True, exist_ok=True)
        a.out.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf8')
    else:
        if a.action == 'capture':
            q = [math.radians(x) for x in a.q_deg]
            if any(not math.isfinite(x) or not model.MODEL['q_min_rad'][i] <= x <= model.MODEL['q_max_rad'][i] for i, x in enumerate(q)):
                p.error('Known reference angles must be finite and inside the model range')
        data = collect(a.port, a.out, baseline, a.samples)
        data['baseline_sha256'] = hashlib.sha256(a.baseline.read_bytes()).hexdigest()
        if a.action == 'capture':
            data.update(schema='so101_model_reference_capture_v1', label=a.label,
                        reference_q_rad=q, reference_q_deg=a.q_deg,
                        reference_angles_source='operator supplied, not independently measured')
        (a.out / 'capture.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf8')
        for row in data['previous_mapping_preview']['joints']:
            print(f"ID{row['id']} {row['name']}: count={row['present_position']} q_deg_preview={row['q_deg_preview']:+.3f} in_model={int(row['in_model_range'])}")
    print('Saved', a.out, '; preview/candidate only; no register writes or motor motion.')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, CalibrationError, KeyboardInterrupt) as error:
        raise SystemExit(str(error))
