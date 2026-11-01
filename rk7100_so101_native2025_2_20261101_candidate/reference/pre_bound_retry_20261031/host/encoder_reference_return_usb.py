"""Small supported return to saved encoder positions; default is offline preview.

No IK/model-zero conversion. Reference is a captured physical pose. No gripper
motion or EEPROM writes. A person/support must carry the arm through final OFF.
This USB host does not provide a hardware watchdog or collision model.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

from hold_current_usb import Hold
from auto_calibrate_usb import USBProtocol
from auto_calibrate_zynq import CalibrationError
from reference_calibration_usb import load_baseline, validate_eeprom, ALL_NAMES

MAX_TRAVEL_COUNTS = 192
STEP_COUNTS = 3
POSITION_TOLERANCE = 16  # ~1.41 degrees; supported pose return, not precision zero metrology.
ADVANCE_TRACKING_TOLERANCE = 12


def reference_targets(path, baseline):
    data = json.loads(path.read_text(encoding='utf8'))
    if (data.get('schema') not in ('so101_encoder_snapshot_v1', 'so101_model_reference_capture_v1')
        or data.get('status') != 'SNAPSHOT_PASS'
        or not data.get('eeprom_unchanged') or not data.get('six_axes_torque_off')
        or not data.get('position_mode') or data.get('register_writes_sent') is not False
        or len(data.get('encoder_counts_median', [])) != 6
        or len(data.get('encoder_counts_spread', [])) != 6
        or max(data['encoder_counts_spread']) > 5
        or data.get('position_space') != 'Present_Position_after_existing_homing_offset'):
        raise CalibrationError('Need a stable original read-only physical-pose capture, not a zero candidate table')
    targets = {}
    for motor, name in enumerate(ALL_NAMES, 1):
        try:
            raw = bytes.fromhex(data['eeprom_before_hex'][str(motor)])
            after = bytes.fromhex(data['eeprom_after_hex'][str(motor)])
            count = data['encoder_counts_median'][motor-1]
        except (KeyError, ValueError) as error:
            raise CalibrationError('Incomplete reference baseline') from error
        validate_eeprom(motor, raw, baseline)
        j = baseline['joint_mapping'][name]
        if raw != after or isinstance(count, bool) or int(count) != count or not j['range_min'] <= count <= j['range_max']:
            raise CalibrationError('Reference count or EEPROM invalid')
        if motor < 6:
            targets[motor] = int(count)
    return targets, data


class EncoderReturn(Hold):
    def __init__(self, bus, folder, baseline, destination, reference):
        super().__init__(bus, folder, baseline)
        self.destination = destination
        self.reference = reference
        self.initial_positions = {}
        self.settled_polls = 0

    def start(self):
        # Check all distances and EEPROM before any WRITE (including fresh hold).
        for motor in range(1, 7):
            raw = self.bus.read(motor, 0, 32) + self.bus.read(motor, 32, 8)
            validate_eeprom(motor, raw, self.baseline)
            if raw.hex() != self.reference['eeprom_before_hex'][str(motor)]:
                raise CalibrationError('Current EEPROM differs from saved physical reference')
            row, _ = self.state(motor, initial=True)
            self.initial_positions[motor] = row['position']
            if motor < 6 and abs(row['position']-self.destination[motor]) > MAX_TRAVEL_COUNTS:
                raise CalibrationError(f'ID{motor}: return exceeds {MAX_TRAVEL_COUNTS} counts; manually approach reference first')
        super().start()
        if any(abs(self.targets[m]-self.destination[m]) > MAX_TRAVEL_COUNTS for m in range(1, 6)):
            raise CalibrationError('Pose changed during supported return preparation')

    def advance(self):
        self.poll()
        for motor in range(1, 6):
            current, _ = self.state(motor)
            goal = self.targets[motor]
            if abs(current['position']-goal) > ADVANCE_TRACKING_TOLERANCE:
                # Never advance the ramp ahead of delayed/stalled feedback.
                continue
            change = max(-STEP_COUNTS, min(STEP_COUNTS, self.destination[motor]-goal))
            if change:
                next_goal = goal+change
                self.write_checked(motor, 42, next_goal.to_bytes(2, 'little'))
                self.targets[motor] = next_goal
        self.poll()
        last = {row['id']: row for row in self.samples[-6:]}
        arrived = all(self.targets[m] == self.destination[m]
            and abs(last[m]['position']-self.destination[m]) <= POSITION_TOLERANCE
            and not last[m]['moving'] and abs(last[m]['speed']) <= 5 for m in range(1, 6))
        self.settled_polls = self.settled_polls+1 if arrived else 0
        return self.settled_polls >= 3


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--port', default='COM12')
    p.add_argument('--execute-supported-return', action='store_true')
    a = p.parse_args()
    if a.out.exists():
        p.error('Use a new output directory')
    baseline = load_baseline(Path(__file__).with_name('right_mapping.json'))
    destination, reference = reference_targets(a.reference, baseline)
    a.out.mkdir(parents=True)
    report = dict(status='OFFLINE_PREVIEW', reference=str(a.reference.resolve()),
        reference_sha256=hashlib.sha256(a.reference.read_bytes()).hexdigest(), destination=destination,
        maximum_travel_counts=MAX_TRAVEL_COUNTS, step_counts=STEP_COUNTS,
        position_tolerance_counts=POSITION_TOLERANCE, hardware_watchdog=False,
        advance_tracking_tolerance_counts=ADVANCE_TRACKING_TOLERANCE,
        model_angles_used=False, eeprom_write_requests=0, gripper_motion=False,
        physical_model_zero_proven=False, automatic_joycon_motion_enabled=False)
    bus = controller = None
    if a.execute_supported_return:
        report['status'] = 'NOT_COMPLETED'
        try:
            bus = USBProtocol(a.port, a.out)
            controller = EncoderReturn(bus, a.out, baseline, destination, reference)
            controller.start()
            print('SUPPORTED ENCODER RETURN ACTIVE; continue physical support through final OFF.', flush=True)
            start = time.monotonic()
            deadline = start+45
            while time.monotonic() < deadline:
                if controller.advance():
                    report.update(status='ENCODER_RETURN_PASS', elapsed_seconds=time.monotonic()-start)
                    break
                time.sleep(.1)
            else:
                raise CalibrationError('Return timed out at 45 s; no further ramp commands')
        except BaseException as error:
            report.update(status='NOT_COMPLETED', error=repr(error))
        finally:
            if controller:
                report.update(initial_positions=controller.initial_positions,
                    last_commanded_positions=controller.targets,
                    maximum_tracking_error=controller.maximum_drift,
                    samples=controller.samples, temperature_policy=controller.thermal.report())
                report['cleanup'] = controller.stop()
                if not report['cleanup']['confirmed']:
                    report['status'] = 'CLEANUP_NOT_CONFIRMED'
            if bus:
                report['packet_log_error'] = bus.log_error
                if bus.log_error:
                    report['status'] = 'NOT_COMPLETED'
                try:
                    bus.close()
                except BaseException as error:
                    report.update(status='NOT_COMPLETED', close_error=repr(error))
    (a.out/'summary.json').write_text(json.dumps(report, indent=2), encoding='utf8')
    print(report['status'], 'targets:', destination, flush=True)
    if a.execute_supported_return and report['status'] != 'ENCODER_RETURN_PASS':
        raise SystemExit(report.get('error', 'Return failed; inspect summary.json'))


if __name__ == '__main__':
    main()
