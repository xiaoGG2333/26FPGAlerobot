"""Meaningful read-only capture and existing-baseline checks; no hardware."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import reference_calibration_usb as ref
from auto_calibrate_usb import packet

BASELINE = ref.load_baseline(Path(ref.__file__).with_name('right_mapping.json'))


class FakeBus:
    def __init__(self, port, folder, fault=None):
        self.log_error = None
        self.fault = fault
        self.requests = []
        self.reads = {}
        self.closed = False
        self.eeprom = {}
        for motor, name in enumerate(ref.ALL_NAMES, 1):
            j = BASELINE['joint_mapping'][name]
            b = bytearray(40)
            b[3:5] = (777).to_bytes(2, 'little')
            b[5] = motor
            b[9:11] = j['range_min'].to_bytes(2, 'little')
            b[11:13] = j['range_max'].to_bytes(2, 'little')
            from auto_calibrate_zynq import encode_signed
            b[31:33] = encode_signed(j['homing_offset_signed'], 11)
            self.eeprom[motor] = b

    def read(self, motor, address, size):
        self.requests.append(packet(motor, 2, bytes((address, size))))
        key = (motor, address)
        self.reads[key] = self.reads.get(key, 0) + 1
        if address in (0, 32):
            b = bytearray(self.eeprom[motor])
            if self.fault == 'changed_eeprom' and motor == 1 and self.reads[key] > 1:
                b[13] ^= 1
            return bytes(b[address:address + size])
        if address == 40:
            return bytes((1 if self.fault == 'torque' and motor == 3 else 0,))
        if address == 56:
            b = bytearray(15)
            j = BASELINE['joint_mapping'][ref.ALL_NAMES[motor - 1]]
            position = (j['range_min'] + j['range_max']) // 2
            if self.fault == 'unstable' and motor == 2:
                position += 10 * self.reads[key]
            b[:2] = position.to_bytes(2, 'little')
            b[6:8] = bytes((122, 31))
            if self.fault == 'alarm' and motor == 4:
                b[9] = 4
            if self.fault == 'speed' and motor == 1:
                b[2] = 1
            return bytes(b)
        raise AssertionError('Unexpected register read')

    def write(self, *args):
        raise AssertionError('READ-ONLY TOOL MUST NEVER WRITE')

    def close(self):
        self.closed = True


class Tests(unittest.TestCase):
    def collect(self, fault=None):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp) / 'capture'
            bus = FakeBus('mock', folder, fault)
            try:
                result = ref.collect('mock', folder, BASELINE, 3, .02, lambda *args: bus)
            finally:
                self.assertTrue(bus.closed)
                self.assertTrue(all(frame[4] == 2 for frame in bus.requests))
                saved = json.loads((folder / 'summary.json').read_text())
                self.assertFalse(saved['register_writes_sent'])
            return result

    def test_read_only_success_preserves_unverified_mapping(self):
        r = self.collect()
        self.assertEqual(r['status'], 'SNAPSHOT_PASS')
        self.assertEqual(r['eeprom_before_hex'], r['eeprom_after_hex'])
        self.assertFalse(r['physical_alignment_verified'])
        self.assertFalse(r['previous_mapping_preview']['automatic_motion_enabled'])
        self.assertNotIn('reference_q_rad', r)

    def test_torque_enabled_rejected(self):
        with self.assertRaises(ref.CalibrationError):
            self.collect('torque')

    def test_alarm_rejected(self):
        with self.assertRaises(ref.CalibrationError):
            self.collect('alarm')

    def test_nonzero_speed_rejected(self):
        with self.assertRaises(ref.CalibrationError):
            self.collect('speed')

    def test_unstable_reference_rejected(self):
        with self.assertRaises(ref.CalibrationError):
            self.collect('unstable')

    def test_eeprom_change_during_capture_rejected(self):
        with self.assertRaises(ref.CalibrationError):
            self.collect('changed_eeprom')

    def test_homing_offset_not_added_twice(self):
        counts = [BASELINE['joint_mapping'][n]['zero_count'] for n in ref.model.NAMES] + [2400]
        rows = ref.preview(BASELINE, counts)['joints']
        self.assertTrue(all(x['q_rad_preview'] == 0 for x in rows))
        counts[0] += 100
        self.assertAlmostEqual(ref.preview(BASELINE, counts)['joints'][0]['q_rad_preview'], -100 / ref.model.CPR)

    def test_unknown_pose_cannot_fit(self):
        r = self.collect()
        r['baseline_sha256'] = 'fixture'
        with self.assertRaises(ValueError):
            ref.fit_references([r, r])
        with self.assertRaises(ValueError):
            ref.fit_references([])

    def test_known_references_fit_candidate_only_and_baseline_bound(self):
        records = []
        for angle in (-.15, .15):
            q = [angle] * 5
            counts = [BASELINE['joint_mapping'][n]['zero_count'] +
                      BASELINE['joint_mapping'][n]['sign'] * angle * ref.model.CPR
                      for n in ref.model.NAMES] + [2400]
            records.append(dict(schema='so101_model_reference_capture_v1', status='SNAPSHOT_PASS',
                                model_fingerprint=ref.model.FINGERPRINT,
                                position_space='Present_Position_after_existing_homing_offset',
                                limits=[list(x) for x in ref.model.LIMITS], homing_offsets=ref.model.OFFSETS,
                                reference_q_rad=q, encoder_counts_median=counts, encoder_counts_spread=[0]*6,
                                eeprom_unchanged=True, six_axes_torque_off=True, position_mode=True,
                                register_writes_sent=False, baseline_sha256='fixture'))
        r = ref.fit_references(records)
        self.assertFalse(r['physical_alignment_verified'])
        self.assertFalse(r['automatic_motion_enabled'])
        self.assertEqual([j['sign'] for j in r['joints']], [-1, 1, 1, -1, 1])
        wrong = copy.deepcopy(records)
        wrong[1]['baseline_sha256'] = 'changed'
        with self.assertRaises(ValueError):
            ref.fit_references(wrong)


if __name__ == '__main__':
    unittest.main()
