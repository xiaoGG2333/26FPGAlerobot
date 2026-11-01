"""Bounded, supported current-position hold; no model-angle or EEPROM writes.

The operator must support the arm throughout. USB has no local watchdog.
STS3215 goal writes can enable torque automatically. Replace only a fresh
current-position goal with all caps already set; never restore a stale goal.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from auto_calibrate_usb import USBProtocol, TemperatureDebounce
from auto_calibrate_zynq import CalibrationError, signed, word
from reference_calibration_usb import load_baseline, validate_eeprom, ALL_NAMES


class Hold:
    def __init__(self, bus, folder, baseline):
        self.bus, self.folder, self.baseline = bus, folder, baseline
        self.original = {}
        self.targets = {}
        self.enabled = set()
        self.modified = set()
        self.thermal = TemperatureDebounce()
        self.last_poll = {}
        self.maximum_drift = {i: 0 for i in range(1, 6)}
        self.samples = []

    def check(self):
        if (self.folder / 'STOP').exists():
            raise CalibrationError('Operator STOP')
        if self.bus.log_error:
            raise CalibrationError('USB packet log failed')
        self.thermal.deadlines()

    def state(self, motor, initial=False):
        self.check()
        now = time.monotonic()
        if motor in self.enabled and now - self.last_poll[motor] > .3:
            raise CalibrationError(f'ID{motor}: monitoring gap over 300 ms')
        mode = self.bus.read(motor, 33, 1)[0]
        data = self.bus.read(motor, 40, 31)
        p = signed(data[16:18], 15)
        row = dict(time=time.time(), id=motor, position=p, torque=data[0], mode=mode,
                   speed=signed(data[18:20], 15), load=signed(data[20:22], 10),
                   voltage=data[22]/10, temperature=data[23], alarm=data[25], moving=data[26])
        j = self.baseline['joint_mapping'][ALL_NAMES[motor-1]]
        if (mode or row['torque'] != int(motor in self.enabled) or row['alarm']
            or not 9 <= row['voltage'] <= 12.6 or not j['range_min'] <= p <= j['range_max']
            or (initial and (row['moving'] or row['speed']))):
            raise CalibrationError(f'ID{motor}: unhealthy or unexpected hold state {row}')
        if motor in self.enabled:
            drift = abs(p - self.targets[motor])
            self.maximum_drift[motor] = max(self.maximum_drift[motor], drift)
            if drift > 16 or abs(row['speed']) > 100:
                raise CalibrationError(f'ID{motor}: current-position hold tracking fault')
            self.last_poll[motor] = time.monotonic()
        self.thermal.observe(motor, row['temperature'], 'hold READ40/31')
        self.samples.append(row)
        return row, data

    def poll(self):
        for motor in range(1, 7):
            self.state(motor)
        self.check()

    def write_checked(self, motor, address, data):
        self.check()
        if self.enabled:
            self.poll()
        lengths = {40:1, 41:1, 42:2, 46:2, 48:2}
        if motor not in range(1, 6) or address not in lengths or len(data) != lengths[address]:
            raise CalibrationError('Hold uses individual whitelisted RAM registers only')
        self.bus.write(motor, address, data)
        if self.bus.read(motor, address, len(data)) != data:
            raise CalibrationError('RAM write readback mismatch')

    def start(self):
        for motor in range(1, 7):
            raw = self.bus.read(motor, 0, 32) + self.bus.read(motor, 32, 8)
            validate_eeprom(motor, raw, self.baseline)
            row, ram = self.state(motor, initial=True)
            self.original[motor] = {'eeprom': raw, 'ram': ram[1:10]}
            if motor < 6:
                self.targets[motor] = row['position']
        while self.thermal.pending:
            self.poll()
        (self.folder/'before.json').write_text(json.dumps({str(k):
            dict(eeprom_hex=v['eeprom'].hex(), ram41_49_hex=v['ram'].hex())
            for k, v in self.original.items()}, indent=2), encoding='utf8')
        # Cap every axis before a goal WRITE can auto-enable the first servo.
        for motor in range(1, 6):
            current, _ = self.state(motor, initial=True)
            if abs(current['position'] - self.targets[motor]) > 5:
                raise CalibrationError('Arm moved during supported hold preparation')
            maximum = min(word(self.original[motor]['eeprom'][16:18]),
                          word(self.original[motor]['ram'][7:9]), 250 if motor == 5 else 400)
            if maximum < 100:
                raise CalibrationError('Existing effective torque cap too low for this hold test')
            if word(self.original[motor]['ram'][3:5]) != 0 or word(self.original[motor]['ram'][5:7]) != 0:
                raise CalibrationError('Supported hold requires existing position time/speed zero; do not modify stale-goal motion controls')
            self.modified.add(motor)
            self.write_checked(motor, 48, maximum.to_bytes(2,'little'))
            self.write_checked(motor, 41, b'\5')
            if self.bus.read(motor, 40, 1) != b'\0':
                raise CalibrationError('Unexpected torque enable while setting non-goal RAM parameters')
        for motor in range(1, 6):
            if self.enabled:
                self.poll()
            row, ram = self.state(motor, initial=True)
            if abs(row['position'] - self.targets[motor]) > 5:
                raise CalibrationError('Current position changed before supported hold')
            self.check()
            # GOAL WRITE itself can enable. Mark before TX, not after its ACK.
            self.enabled.add(motor)
            self.last_poll[motor] = time.monotonic()
            self.targets[motor] = row['position']
            self.bus.write(motor, 42, row['position'].to_bytes(2,'little'))
            if self.bus.read(motor, 42, 2) != row['position'].to_bytes(2,'little'):
                raise CalibrationError('Fresh current-position goal readback mismatch')
            if self.bus.read(motor, 40, 1) != b'\1':
                raise CalibrationError('Expected goal-write auto-enable was not confirmed; unsupported behavior')
        self.poll()

    def stop(self):
        errors, off, restored = [], [], []
        # OFF all modified axes; NEVER restore a stale goal (it auto-enables).
        for motor in sorted(self.modified):
            for attempt in range(3):
                try:
                    self.bus.write(motor, 40, b'\0')
                    if self.bus.read(motor, 40, 1) != b'\0':
                        raise CalibrationError('Torque OFF not confirmed')
                    off.append(motor)
                    break
                except BaseException as error:
                    if attempt == 2:
                        errors.append(f'ID{motor} OFF: {error!r}')
        self.enabled.clear()
        for motor in sorted(self.modified):
            if motor not in off:
                continue
            try:
                ram = self.original[motor]['ram']
                for address, data in ((41,ram[:1]), (48,ram[7:9])):
                    self.bus.write(motor, address, data)
                    if self.bus.read(motor, address, len(data)) != data:
                        raise CalibrationError('Non-goal RAM restore readback mismatch')
                restored.append(motor)
            except BaseException as error:
                errors.append(f'ID{motor} RAM restore: {error!r}')
            finally:
                # Attempt final OFF even when a restore readback/ACK failed.
                # Restoration errors must not skip the final OFF transaction.
                for attempt in range(3):
                    try:
                        self.bus.write(motor, 40, b'\0')
                        if self.bus.read(motor, 40, 1) != b'\0':
                            raise CalibrationError('Final torque OFF after RAM restoration not confirmed')
                        break
                    except BaseException as error:
                        if attempt == 2:
                            errors.append(f'ID{motor} final OFF: {error!r}')
        final = []
        for motor in sorted(self.original):
            try:
                raw = self.bus.read(motor, 0, 32) + self.bus.read(motor, 32, 8)
                torque = self.bus.read(motor, 40, 1)[0]
                item = dict(id=motor, torque=torque, mode=raw[33], eeprom_unchanged=raw == self.original[motor]['eeprom'])
                final.append(item)
                if torque or raw[33] or not item['eeprom_unchanged']:
                    raise CalibrationError('Final baseline/OFF check failed')
            except BaseException as error:
                errors.append(f'ID{motor} final: {error!r}')
        return dict(confirmed=not errors, off_confirmed=off, non_goal_ram_restored=restored,
                    old_goal_restored=False, final=final, errors=errors)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM12')
    p.add_argument('--seconds', type=float, default=10)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--run-supported-test', action='store_true',
                   help='Explicit bounded test; operator supports throughout; goal WRITE auto-enables')
    a = p.parse_args()
    if not 2 <= a.seconds <= 30 or a.out.exists():
        p.error('Use a new output directory and duration 2..30 seconds')
    if not a.run_supported_test:
        p.error('No torque operation by default; use --run-supported-test only with continuous physical support')
    baseline = load_baseline(Path(__file__).with_name('right_mapping.json'))
    a.out.mkdir(parents=True)
    report = dict(status='NOT_COMPLETED', hardware_watchdog=False, eeprom_write_requests=0,
                  requested_hold_seconds=a.seconds, model_angles_used=False,
                  physical_alignment_verified=False, automatic_motion_enabled=False)
    bus = hold = None
    try:
        bus = USBProtocol(a.port, a.out)
        hold = Hold(bus, a.out, baseline)
        hold.start()
        print('HOLD ACTIVE: five measured positions; gripper OFF; continue physical support.', flush=True)
        print('Targets:', hold.targets, flush=True)
        start = time.monotonic()
        while time.monotonic()-start < a.seconds:
            hold.poll()
            time.sleep(.02)
        report.update(status='HOLD_TEST_PASS', hold_elapsed_seconds=time.monotonic()-start)
    except BaseException as error:
        report['error'] = repr(error)
    finally:
        if hold:
            report.update(targets=hold.targets, maximum_drift_counts=hold.maximum_drift,
                          temperature_policy=hold.thermal.report(), samples=hold.samples)
            report['cleanup'] = hold.stop()
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
        print('HOLD ENDED:', report['status'], '; OFF/restore:', report.get('cleanup',{}).get('confirmed'), flush=True)
    if report['status'] != 'HOLD_TEST_PASS':
        raise SystemExit(report.get('error', 'Hold verification failed; inspect summary.json'))


if __name__ == '__main__':
    main()
