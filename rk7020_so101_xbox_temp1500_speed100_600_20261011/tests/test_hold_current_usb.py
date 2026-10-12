"""Fault-injection tests for bounded supported hold; no serial port opened."""
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import hold_current_usb as hc
from auto_calibrate_zynq import encode_signed

BASELINE = hc.load_baseline(Path(hc.__file__).with_name('right_mapping.json'))


class Bus:
    def __init__(self):
        self.log_error = None
        self.writes = []
        self.memory = {}
        self.enable_checks = []
        self.failure = None
        for motor, name in enumerate(hc.ALL_NAMES, 1):
            j = BASELINE['joint_mapping'][name]
            b = bytearray(80)
            b[3:5] = (777).to_bytes(2, 'little')
            b[5] = motor
            b[9:11] = j['range_min'].to_bytes(2, 'little')
            b[11:13] = j['range_max'].to_bytes(2, 'little')
            b[16:18] = (1000).to_bytes(2, 'little')
            b[31:33] = encode_signed(j['homing_offset_signed'], 11)
            b[41] = 10
            b[42:44] = (1234).to_bytes(2, 'little')  # Stale target must be replaced.
            b[48:50] = (400).to_bytes(2, 'little')
            position = (j['range_min'] + j['range_max']) // 2 if motor != 5 else 146
            b[56:58] = position.to_bytes(2, 'little')
            b[62:64] = bytes((122, 31))
            self.memory[motor] = b

    def read(self, motor, address, count):
        return bytes(self.memory[motor][address:address+count])

    def write(self, motor, address, data):
        assert motor in range(1, 6) and address >= 40
        self.writes.append((motor, address, bytes(data)))
        self.memory[motor][address:address+len(data)] = data
        if address <= 42 and address+len(data) >= 44:
            # Real STS3215 behavior absent from the first mock: goal writes
            # auto-enable, including contiguous blocks that span address42.
            self.memory[motor][40] = 1
            self.enable_checks.append(hc.word(self.memory[motor][42:44]) == hc.word(self.memory[motor][56:58])
                 and all(hc.word(b[48:50]) <= (250 if i == 5 else 400)
                         for i,b in self.memory.items() if i < 6))
        if self.failure == (motor, address, bytes(data)):
            self.failure = None
            raise hc.CalibrationError('Injected lost acknowledgement after WRITE took effect')


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.bus = Bus()
        self.original = {i: bytes(b) for i, b in self.bus.memory.items()}
        self.hold = hc.Hold(self.bus, self.folder, BASELINE)

    def tearDown(self):
        self.temp.cleanup()

    def assert_restored(self):
        result = self.hold.stop()
        self.assertTrue(result['confirmed'], result)
        for motor, raw in self.original.items():
            self.assertEqual(self.bus.memory[motor][40], 0)
            self.assertEqual(bytes(self.bus.memory[motor][:40]), raw[:40])
            self.assertEqual(bytes(self.bus.memory[motor][41:42]), raw[41:42])
            self.assertEqual(bytes(self.bus.memory[motor][44:50]), raw[44:50])
        self.assertFalse(result['old_goal_restored'])
        return result

    def test_fresh_goal_auto_enable_after_all_caps_set(self):
        self.hold.start()
        self.assertEqual(len(self.bus.enable_checks), 5)
        self.assertTrue(all(self.bus.enable_checks))
        self.assertEqual(self.bus.memory[6][40], 0)
        self.assertTrue(all(motor != 6 for motor, _, _ in self.bus.writes))
        self.assert_restored()

    def test_hold_uses_encoders_even_outside_unverified_model_range(self):
        self.hold.start()
        self.assertEqual(self.hold.targets[5], 146)
        self.assert_restored()

    def test_eeprom_mismatch_rejects_before_writes(self):
        self.bus.memory[2][31] ^= 1
        with self.assertRaises(hc.CalibrationError):
            self.hold.start()
        self.assertEqual(self.bus.writes, [])

    def test_goal_write_lost_ack_still_restored(self):
        self.bus.failure = (3, 42, self.bus.read(3,56,2))
        with self.assertRaises(hc.CalibrationError):
            self.hold.start()
        self.assertIn(3, self.hold.modified)
        self.assert_restored()

    def test_torque_enable_lost_ack_still_off_first(self):
        self.bus.failure = (3, 42, self.bus.read(3,56,2))
        with self.assertRaises(hc.CalibrationError):
            self.hold.start()
        self.assertEqual(self.bus.memory[3][40], 1)
        self.bus.writes.clear()
        self.assert_restored()
        self.assertEqual(self.bus.writes[:5], [(i,40,b'\0') for i in range(1,6)])

    def test_cleanup_never_restores_stale_goal_or_spanning_block(self):
        self.hold.start()
        self.bus.writes.clear()
        self.assert_restored()
        self.assertTrue(all(address != 42 and not (address <= 42 < address+len(data))
                            for _,address,data in self.bus.writes))
        for motor in range(1,6):
            self.assertEqual([x for x in self.bus.writes if x[0] == motor][-1], (motor,40,b'\0'))

    def test_restore_ack_failure_still_attempts_final_off(self):
        self.hold.start()
        self.bus.writes.clear()
        self.bus.failure = (2,41,self.original[2][41:42])
        result = self.hold.stop()
        self.assertFalse(result['confirmed'])
        for motor in range(1,6):
            self.assertEqual([x for x in self.bus.writes if x[0] == motor][-1], (motor,40,b'\0'))
            self.assertEqual(self.bus.memory[motor][40],0)

    def test_tracking_fault_stops_and_restores(self):
        self.hold.start()
        self.bus.memory[2][56:58] = (self.hold.targets[2] + 17).to_bytes(2,'little')
        with self.assertRaises(hc.CalibrationError):
            self.hold.poll()
        self.assert_restored()

    def test_monitor_gap_stops_and_restores(self):
        self.hold.start()
        self.hold.last_poll[1] = time.monotonic()-1
        with self.assertRaises(hc.CalibrationError):
            self.hold.poll()
        self.assert_restored()

    def test_alarm_stops_and_restores(self):
        self.hold.start()
        self.bus.memory[4][65] = 4
        with self.assertRaises(hc.CalibrationError):
            self.hold.poll()
        self.assert_restored()

    def test_persistent_temperature_stops_and_restores(self):
        self.hold.start()
        self.hold.thermal.observe(4, 80, 'mock')
        self.hold.thermal.pending[4]['started_monotonic'] -= 2
        with self.assertRaises(hc.CalibrationError):
            self.hold.poll()
        self.assert_restored()

    def test_stop_file_stops_and_restores(self):
        self.hold.start()
        (self.folder/'STOP').touch()
        with self.assertRaises(hc.CalibrationError):
            self.hold.poll()
        self.assert_restored()


if __name__ == '__main__':
    unittest.main()
