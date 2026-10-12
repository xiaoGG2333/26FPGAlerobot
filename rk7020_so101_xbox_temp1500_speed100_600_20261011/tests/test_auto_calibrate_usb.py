"""No-device binary bus mocks for direct USB calibration guards and cleanup."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import auto_calibrate_usb as usb
from auto_calibrate_zynq import Calibrator, CalibrationError, encode_signed, signed
from vendor_calibration_math import MOTOR_IDS


class Clock:
    def __init__(self): self.now = 0.
    def monotonic(self): return self.now
    def time(self): return self.now + 1000000
    def sleep(self, amount): self.now += amount


class SerialMock:
    def __init__(self, clock):
        self.clock = clock
        self.rx = bytearray()
        self.frames = []
        self.closed = False
        self.echo = False
        self.alter = None
        self.ignore_off = set()
        self.fail = None
        self.registers = {}
        self.physical = {i: 2000 for i in range(1, 7)}
        for motor in range(1, 7):
            data = bytearray(80)
            data[3:5] = (777).to_bytes(2, 'little')
            data[5] = motor
            data[9:11] = (1000).to_bytes(2, 'little')
            data[11:13] = (3000).to_bytes(2, 'little')
            data[16:18] = (1000).to_bytes(2, 'little')
            data[21:24] = b'\x10\0\0'
            data[26:28] = b'\x01\x01'
            data[34:37] = bytes((70, 140, 90))
            data[41] = 5
            data[42:44] = (2500).to_bytes(2, 'little')  # Deliberately stale goal.
            data[48:50] = (1000).to_bytes(2, 'little')
            data[55] = 1
            data[62:64] = bytes((120, 30))
            self.registers[motor] = data

    @property
    def in_waiting(self): return len(self.rx)
    def read(self, count):
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result
    def _sync(self, motor):
        data = self.registers[motor]
        offset = signed(data[31:33], 11)
        position = self.physical[motor] if data[33] == 1 else (self.physical[motor] - offset) % 4096
        data[56:58] = position.to_bytes(2, 'little')
        data[58:60] = data[46:48] if data[33] == 1 and data[40] else b'\0\0'

    def write(self, frame):
        frame = bytes(frame)
        self.clock.now += .001
        if sum(frame[2:]) & 255 != 255:
            raise AssertionError('Request checksum invalid')
        motor, operation, address = frame[2], frame[4], frame[5]
        self.frames.append(frame)
        if self.fail and self.fail(operation, motor, address, frame):
            return len(frame)
        data = self.registers[motor]
        if operation == 2:
            self._sync(motor)
            value = bytes(data[address:address + frame[6]])
        else:
            value = b''
            written = frame[6:-1]
            if not (address == 40 and written == b'\0' and motor in self.ignore_off):
                if address < 40 and (data[40] or data[55]):
                    raise AssertionError('EEPROM write while locked or TorqueON')
                data[address:address + len(written)] = written
                if address == 33:
                    data[41] = 0  # Confirmed STS3215 firmware behavior.
                if (address == 42 and data[33] == 0) or (address == 46 and data[33] == 1):
                    data[40] = 1  # Includes Mode1 Speed0 auto-enable.
        response = usb.packet(motor, 0, value)
        if self.alter:
            alter, self.alter = self.alter, None
            response = alter(response)
        if self.echo:
            self.rx.extend(frame)
        self.rx.extend(response)
        return len(frame)
    def close(self): self.closed = True


class USBTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.clock = Clock()
        self.device = SerialMock(self.clock)
        self.patches = [patch.object(usb.time, 'monotonic', self.clock.monotonic),
                        patch.object(usb.time, 'sleep', self.clock.sleep),
                        patch.object(usb.time, 'time', self.clock.time)]
        for p in self.patches: p.start()
        self.bridge = None
    def tearDown(self):
        if self.bridge:
            self.device.ignore_off.clear()
            self.device.fail = None
            try: self.bridge.close()
            except Exception: pass
        for p in reversed(self.patches): p.stop()
        self.temp.cleanup()
    def open(self):
        self.bridge = usb.Bridge('MOCK-NOT-A-PORT', self.folder, lambda *a, **k: self.device)
        self.bridge.start()
        return self.bridge
    def ready(self):
        b = self.open()
        for motor in range(1, 7):
            b.byte(motor, 55, 0)
            b.uint(motor, 16, 600)
            b.uint(motor, 48, 250 if motor in (5, 6) else 400)
            b.byte(motor, 41, 5)
            b.uint(motor, 46, 0)
        return b
    def running(self, motor=4):
        b = self.ready()
        b.byte(motor, 33, 1)
        b.byte(motor, 41, 5)
        b.uint(motor, 46, 0)
        b.byte(motor, 40, 1)
        b.write(motor, 46, encode_signed(100, 15))
        return b
    @staticmethod
    def writes(frames): return [x for x in frames if x[4] == 3]
    def assert_off(self):
        self.assertTrue(all(data[40] == 0 for data in self.device.registers.values()))

    def test_protocol_echo_drain_and_strict_response(self):
        protocol = usb.USBProtocol('MOCK', self.folder, lambda *a, **k: self.device)
        self.device.rx.extend(b'old data')
        self.device.echo = True
        self.assertEqual(protocol.read(1, 3, 2), (777).to_bytes(2, 'little'))
        record = json.loads((self.folder / 'usb_packets.jsonl').read_text().splitlines()[-1])
        self.assertEqual(bytes.fromhex(record['drained']), b'old data')
        protocol.close()

    def test_protocol_bad_checksum_id_length_and_no_retry(self):
        protocol = usb.USBProtocol('MOCK', self.folder, lambda *a, **k: self.device)
        for alter in (lambda x: x[:-1] + bytes((x[-1] ^ 1,)),
                      lambda x: usb.packet(2, 0, x[5:-1]),
                      lambda x: usb.packet(1, 0, b'\x01')):
            self.device.alter = alter
            count = len(self.device.frames)
            with self.assertRaises(CalibrationError): protocol.read(1, 3, 2)
            self.assertEqual(len(self.device.frames), count + 1)
        protocol.close()

    def test_protocol_timeout_and_nonzero_write_not_retried(self):
        protocol = usb.USBProtocol('MOCK', self.folder, lambda *a, **k: self.device)
        self.device.fail = lambda *a: True
        with self.assertRaises(CalibrationError): protocol.write(4, 46, (100).to_bytes(2, 'little'))
        self.assertEqual(len(self.device.frames), 1)
        protocol.close()

    def test_start_readonly_complete_backup_and_baseline_rejection(self):
        b = self.open()
        self.assertFalse(self.writes(self.device.frames))
        self.assertEqual(len(b.original), 6)
        self.assertTrue((self.folder / 'usb_transport_before.json').exists())
        b.finish(False)
        self.assert_off()

    def test_start_bad_mode_no_settings_after_incomplete_backup(self):
        self.device.registers[4][33] = 1
        self.bridge = usb.Bridge('MOCK', self.folder, lambda *a, **k: self.device)
        with self.assertRaises(CalibrationError): self.bridge.start()
        self.bridge.finish(False)
        self.assertTrue(all(x[5] == 40 and x[6:-1] == b'\0' for x in self.writes(self.device.frames)))

    def test_hold_caps_all_six_before_goal_and_no_old_goal_restore(self):
        b = self.open()
        with self.assertRaises(CalibrationError): b.hold(1)
        self.assertFalse(any(x[5] == 42 for x in self.writes(self.device.frames)))
        self.assert_off()

    def test_atomic_hold_goal_auto_enable_and_offset_switch(self):
        self.device.registers[4][31:33] = encode_signed(100, 11)
        b = self.ready()
        b.hold(4)
        self.assertEqual(int.from_bytes(self.device.registers[4][42:44], 'little'), 1900)
        self.assertEqual(self.device.registers[4][40], 1)
        b.byte(4, 40, 0)
        b.uint(4, 9, 0)
        b.uint(4, 11, 4095)
        b.uint(4, 31, 0)
        self.assertEqual(int.from_bytes(b.read(4, 56, 2), 'little'), 2000)
        b.byte(4, 33, 1)
        self.assertEqual(b.read(4, 41, 1), b'\0')
        b.byte(4, 41, 5)
        b.uint(4, 46, 0)
        b.write(4, 46, encode_signed(-100, 15))
        before = len(self.device.frames)
        b.finish(False)
        writes = self.writes(self.device.frames[before:])
        self.assertEqual((writes[0][2], writes[0][5], writes[0][6:-1]), (4, 40, b'\0'))
        self.assertFalse(any(x[5] == 42 for x in writes))
        self.assertEqual(self.device.registers[4][:40], b.original[4][:40])
        self.assert_off()

    def test_speed_requires_mode_acceleration_and_caps(self):
        for failure in ('mode', 'acc', 'cap', 'too_fast'):
            with self.subTest(failure=failure):
                # Fresh fixture for each automatic rollback.
                if self.bridge: self.bridge.close()
                self.device = SerialMock(self.clock)
                b = self.ready()
                b.byte(4, 33, 1)
                b.byte(4, 41, 5)
                b.uint(4, 46, 0)
                if failure == 'mode': self.device.registers[4][33] = 0
                if failure == 'acc': self.device.registers[4][41] = 0
                if failure == 'cap': self.device.registers[4][48:50] = (401).to_bytes(2, 'little')
                before = len(self.device.frames)
                with self.assertRaises(CalibrationError): b.write(4, 46, encode_signed(301 if failure == 'too_fast' else 100, 15))
                self.assertFalse(any(x[5] == 46 and x[6:-1] != b'\0\0' for x in self.writes(self.device.frames[before:])))
                self.assert_off()

    def test_eeprom_and_goal_allowlist_blocked_while_torque_on(self):
        b = self.ready()
        b.hold(1)
        with self.assertRaises(CalibrationError): b.uint(1, 31, 10)
        self.assert_off()
        self.assertEqual(self.device.registers[1][31:33], b'\0\0')

    def test_temperature_failure_immediate_off_first_and_rollback(self):
        b = self.running()
        old = {m: bytes(v[:40]) for m, v in b.original.items()}
        self.device.registers[4][63] = 73
        before = len(self.device.frames)
        with self.assertRaises(CalibrationError): b.keep()
        writes = self.writes(self.device.frames[before:])
        self.assertEqual((writes[0][2], writes[0][5]), (4, 40))
        self.assert_off()
        for m in range(1, 7): self.assertEqual(bytes(self.device.registers[m][:40]), old[m])
        self.assertFalse(b.committed)

    def test_single_hot_frame_off_rollback_then_spaced_readonly_recheck(self):
        b = self.running()
        def one_high(frame):
            data = bytearray(frame[5:-1]); data[7] = 77
            return usb.packet(4, 0, data)
        self.device.alter = one_high
        before = len(self.device.frames)
        with self.assertRaises(usb.TemperatureNotReproduced): b.keep()
        frames = self.device.frames[before:]
        writes = self.writes(frames)
        self.assertEqual((writes[0][2], writes[0][5], writes[0][6:-1]), (4, 40, b'\0'))
        recheck_index = next(i for i, x in enumerate(frames) if x[4] == 2 and x[5:7] == bytes((63, 1)))
        self.assertFalse(self.writes(frames[recheck_index:]))
        self.assert_off()
        event = b.temperature_recheck
        self.assertEqual(event['classification'], 'HIGH_NOT_REPRODUCED_AFTER_CONFIRMED_STOP')
        self.assertGreaterEqual(event['recheck_duration_s'], 2)
        self.assertEqual(len(event['samples']), 5)
        self.assertFalse(event['initial_reading_proven_false'])
        self.assertFalse(b.active or b.committed)
        with self.assertRaises(CalibrationError): b.write(4, 46, encode_signed(100, 15))

    def test_pair_only_hot_frame_uses_calibrator_validator(self):
        b = self.running()
        runner = Calibrator(b, self.folder)
        original_read = b.bus.read
        def hot_pair(motor, address, size):
            data = original_read(motor, address, size)
            if address == 62 and size == 2:
                return data[:1] + bytes((77,))
            return data
        with patch.object(b.bus, 'read', hot_pair):
            with self.assertRaises(usb.TemperatureNotReproduced): runner.feedback(4)
        self.assert_off()
        self.assertEqual(b.temperature_recheck['initial_source'], 'READ62/2')
        self.assertFalse(runner.endpoints or runner.results or b.committed)

    def test_critical_alarm_or_inconsistent_cool_values_do_not_allow_retry(self):
        for failure in ('critical', 'alarm', 'inconsistent'):
            with self.subTest(failure=failure):
                if self.bridge: self.bridge.close()
                self.device = SerialMock(self.clock)
                b = self.running()
                if failure == 'critical': self.device.registers[4][63] = 80
                elif failure == 'alarm': self.device.registers[4][65] = 4
                else:
                    def one_high(frame):
                        data = bytearray(frame[5:-1]); data[7] = 77
                        self.device.registers[4][63] = 40
                        return usb.packet(4, 0, data)
                    self.device.alter = one_high
                try:
                    b.keep()
                    self.fail('Fault expected')
                except usb.TemperatureNotReproduced:
                    self.fail('Uncertain fault must never allow a new-session retry')
                except CalibrationError:
                    pass
                self.assert_off()
                self.assertFalse(b.committed)

    def test_external_stop_and_recheck_log_fault_never_allow_retry(self):
        b = self.running()
        def one_high(frame):
            data = bytearray(frame[5:-1]); data[7] = 77
            return usb.packet(4, 0, data)
        self.device.alter = one_high
        b.external_stop_path = self.folder / 'PARENT_STOP'
        original_finish = b.finish
        def finish_then_stop(commit):
            result = original_finish(commit)
            b.external_stop_path.write_text('stop')
            return result
        with patch.object(b, 'finish', finish_then_stop):
            with self.assertRaises(CalibrationError) as raised: b.keep()
        self.assertNotIsInstance(raised.exception, usb.TemperatureNotReproduced)
        self.assert_off()

    def test_high_with_unconfirmed_off_has_no_recheck_or_settings(self):
        b = self.running()
        self.device.registers[4][63] = 77
        self.device.ignore_off.add(4)
        before = len(self.device.frames)
        with self.assertRaises(CalibrationError) as raised: b.keep()
        self.assertNotIsInstance(raised.exception, usb.TemperatureNotReproduced)
        frames = self.device.frames[before:]
        self.assertFalse(any(x[4] == 2 and x[5] == 63 for x in frames))
        self.assertTrue(all(x[5] == 40 for x in self.writes(frames) if x[2] == 4))
        self.assertFalse(b.committed)

    def test_recovered_cleanup_transport_error_still_blocks_new_session(self):
        b = self.running()
        def one_high(frame):
            data = bytearray(frame[5:-1]); data[7] = 77
            return usb.packet(4, 0, data)
        self.device.alter = one_high
        failed = []
        def one_failed_off(op, motor, address, frame):
            if op == 3 and motor == 4 and address == 40 and not failed:
                failed.append(True); return True
            return False
        self.device.fail = one_failed_off
        with self.assertRaises(CalibrationError) as raised: b.keep()
        self.assertNotIsInstance(raised.exception, usb.TemperatureNotReproduced)
        self.assertTrue(b.cleanup_details['confirmed'] and b.cleanup_details['errors'])
        self.assert_off()

    def test_cleanup_output_failure_blocks_temperature_retry(self):
        b = self.running()
        def one_high(frame):
            data=bytearray(frame[5:-1]); data[7]=77
            return usb.packet(4,0,data)
        self.device.alter=one_high
        original_write=Path.write_text
        def fail_cleanup(path,*args,**kwargs):
            if path.name=='usb_cleanup.json': raise OSError('mock cleanup output failure')
            return original_write(path,*args,**kwargs)
        with patch.object(Path,'write_text',fail_cleanup):
            with self.assertRaises(CalibrationError) as raised: b.keep()
        self.assertNotIsInstance(raised.exception,usb.TemperatureNotReproduced)
        self.assertTrue(b.cleanup_output_error)
        self.assert_off()

    def test_invalid_encoder_is_never_classified_as_temperature_retry(self):
        for size in (6, 15):
            with self.subTest(size=size):
                if self.bridge: self.bridge.close()
                self.device=SerialMock(self.clock)
                b=self.ready()
                self.device.registers[3][63]=77
                def invalid_position(frame):
                    data=bytearray(frame[5:-1]); data[:2]=(5000).to_bytes(2,'little')
                    return usb.packet(3,0,data)
                self.device.alter=invalid_position
                with self.assertRaises(CalibrationError) as raised: b.read(3,56,size)
                self.assertNotIsInstance(raised.exception,usb.TemperatureNotReproduced)
                self.assertIsNone(b.temperature_recheck)
                self.assert_off()

    def test_stop_file_log_failure_gap_and_second_axis_all_off(self):
        for failure in ('STOP', 'log', 'gap', 'hold_other'):
            with self.subTest(failure=failure):
                if self.bridge: self.bridge.close()
                self.device = SerialMock(self.clock)
                b = self.running()
                if failure == 'STOP': b.stop_path.write_text('stop')
                elif failure == 'log': b.bus.log_error = 'disk full'
                elif failure == 'gap': self.clock.sleep(.31)
                with self.assertRaises(CalibrationError):
                    b.hold(2) if failure == 'hold_other' else b.keep()
                self.assert_off()
                if b.stop_path.exists(): b.stop_path.unlink()

    def test_unconfirmed_off_axis_never_mode_speed_eeprom_or_commit(self):
        b = self.running()
        self.device.ignore_off.add(4)
        before = len(self.device.frames)
        with self.assertRaises(CalibrationError): b.finish(False)
        writes = [x for x in self.writes(self.device.frames[before:]) if x[2] == 4]
        self.assertTrue(writes and all(x[5] == 40 and x[6:-1] == b'\0' for x in writes))
        self.assertEqual(b.active_axis, 4)
        self.assertTrue(b.fault)
        with self.assertRaises(CalibrationError): b.finish(True)

    def test_stop_no_mode1_zero_auto_enable_and_repeat_does_not_write(self):
        b = self.running()
        before = len(self.device.frames)
        b.finish(False)
        writes = self.writes(self.device.frames[before:])
        first_six = writes[:6]
        self.assertEqual({x[2] for x in first_six}, set(range(1, 7)))
        self.assertTrue(all(x[5] == 40 and x[6:-1] == b'\0' for x in first_six))
        count = len(self.device.frames)
        b.finish(False)
        self.assertEqual(len(self.device.frames), count)
        self.assert_off()

    def test_commit_blocked_without_full_verified_results(self):
        b = self.ready()
        with self.assertRaises(CalibrationError): b.finish(True)
        self.assertFalse(b.committed)
        b.finish(False)

    def complete_runner(self):
        b = self.open()
        runner = Calibrator(b, self.folder, 100, 400)
        b.calibrator = runner
        with contextlib.redirect_stdout(io.StringIO()): runner.backup()
        def explore(name, direction, label):
            motor = MOTOR_IDS[name]
            endpoint = 3000 if direction > 0 else 1000
            self.device.physical[motor] = endpoint
            runner.endpoints.append({'name': name, 'id': motor, 'label': label,
                                     'expected_directional_travel': 2000})
            return endpoint, 2000
        def midpoint(name, aux): self.device.physical[MOTOR_IDS[name]] = aux['physical_zero']
        runner.explore = explore
        runner.midpoint = midpoint
        return b, runner

    def test_complete_calibrator_workflow_commit_preserves_new_limits_only(self):
        b, runner = self.complete_runner()
        with contextlib.redirect_stdout(io.StringIO()): results = runner.run()
        self.assertEqual(set(results), set(MOTOR_IDS))
        self.assertTrue(runner.commit_confirmed and b.committed)
        self.assert_off()
        for name, motor in MOTOR_IDS.items():
            data = self.device.registers[motor]
            self.assertEqual(signed(data[31:33], 11), results[name]['homing_offset'])
            self.assertEqual(int.from_bytes(data[9:11], 'little'), results[name]['range_min'])
            self.assertEqual(data[16:18], b.original[motor][16:18])
            self.assertEqual(data[34:37], b.original[motor][34:37])
            self.assertEqual(data[55], b.original[motor][55])
        count = len(self.device.frames)
        b.finish(False)
        self.assertEqual(len(self.device.frames), count)

    def test_committed_final_json_error_retains_candidate_and_no_rollback(self):
        b, runner = self.complete_runner()
        original_write = Path.write_text
        def failing_write(path, *args, **kwargs):
            if path.name == 'rk7020_right_so101.json':
                raise OSError('mock disk full for final JSON only')
            return original_write(path, *args, **kwargs)
        with patch.object(Path, 'write_text', failing_write), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(OSError): runner.run()
        self.assertTrue(runner.commit_confirmed and b.committed)
        self.assertTrue((self.folder / 'candidate_calibration.json').exists())
        count = len(self.device.frames)
        b.finish(False)
        self.assertEqual(len(self.device.frames), count)
        self.assert_off()

    def test_cleanup_output_failure_cannot_change_confirmed_commit(self):
        b, runner = self.complete_runner()
        original_write = Path.write_text
        def failing_write(path, *args, **kwargs):
            if path.name == 'usb_cleanup.json':
                raise OSError('mock cleanup output error')
            return original_write(path, *args, **kwargs)
        with patch.object(Path, 'write_text', failing_write), contextlib.redirect_stdout(io.StringIO()):
            runner.run()
        self.assertTrue(runner.commit_confirmed and b.committed)
        self.assert_off()

    def test_actual_log_io_failure_stops_and_does_not_block_off(self):
        b = self.running()
        class FailingLog:
            def write(self, _): raise OSError('mock log disk full')
            def close(self): pass
        b.bus.log.close()
        b.bus.log = FailingLog()
        before = len(self.device.frames)
        with self.assertRaises(CalibrationError): b.keep()
        self.assertTrue(b.bus.log_error)
        self.assertEqual(self.writes(self.device.frames[before:])[0][5], 40)
        self.assert_off()


if __name__ == '__main__': unittest.main()
