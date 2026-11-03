"""No-device tests for the explicit 45 C / 1.5 s user-selected policy."""
import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import auto_calibrate_usb as usb
from auto_calibrate_zynq import Calibrator, CalibrationError
import test_auto_calibrate_usb as fixtures
SerialMock=fixtures.SerialMock


class DebounceTests(unittest.TestCase):
    setUp = fixtures.USBTests.setUp
    tearDown = fixtures.USBTests.tearDown
    open = fixtures.USBTests.open
    ready = fixtures.USBTests.ready
    running = fixtures.USBTests.running
    assert_off = fixtures.USBTests.assert_off
    writes = staticmethod(fixtures.USBTests.writes)

    def policy(self, running=False):
        b = self.running() if running else self.ready()
        b.temperature_guard = usb.TemperatureDebounce()
        return b

    def test_exact_threshold_and_exact_time_boundary(self):
        guard=usb.TemperatureDebounce()
        guard.observe(3,45,'test'); self.assertFalse(guard.pending)
        guard.observe(3,46,'test'); self.clock.sleep(1.5)
        guard.observe(3,150,'test')  # exactly 1.5 is permitted
        self.clock.sleep(.00001)
        with self.assertRaises(CalibrationError): guard.observe(3,46,'test')

    def test_short_high_then_normal_clears_only_that_axis(self):
        guard=usb.TemperatureDebounce()
        guard.observe(3,150,'test'); guard.observe(4,70,'test')
        self.clock.sleep(.08); guard.observe(3,33,'test')
        self.assertNotIn(3,guard.pending); self.assertIn(4,guard.pending)
        self.assertAlmostEqual(guard.events[0]['duration_s'],.08)
        self.assertFalse(guard.events[0]['initial_reading_proven_false'])

    def test_live_short_high_keeps_real_bytes_no_stop_and_no_protection_writes(self):
        b=self.policy(running=True)
        self.device.registers[4][63]=150
        before=len(self.device.frames)
        data=b.read(4,56,15)
        self.assertEqual(data[7],150)
        self.clock.sleep(.1); self.device.registers[4][63]=33
        b.keep()
        self.assertTrue(b.active); self.assertFalse(b.temperature_guard.pending)
        self.assertTrue(b.temperature_guard.events)
        self.assertFalse(self.writes(self.device.frames[before:]))
        self.assertEqual(self.device.registers[4][40],1)

    def test_persistent_high_off_first_and_eeprom_rollback(self):
        b=self.policy(running=True)
        self.device.registers[4][63]=150
        b.keep(); before=len(self.device.frames)
        with self.assertRaises(CalibrationError):
            for _ in range(12):
                self.clock.sleep(.15); b.keep()
        writes=self.writes(self.device.frames[before:])
        self.assertEqual((writes[0][2],writes[0][5],writes[0][6:-1]),(4,40,b'\0'))
        self.assert_off()
        for motor in range(1,7):
            self.assertEqual(bytes(self.device.registers[motor][:40]),b.original[motor][:40])
        self.assertFalse(b.committed)

    def test_hot_support_axis_is_polled_while_other_axis_moves(self):
        b=self.policy(running=True)
        self.device.registers[3][63]=70
        b.read(3,63,1); before=len(self.device.frames)
        self.clock.sleep(.1); self.device.registers[3][63]=33; b.keep()
        self.assertFalse(b.temperature_guard.pending)
        self.assertTrue(any(x[2]==3 and x[4]==2 and x[5:7]==bytes((56,15)) for x in self.device.frames[before:]))
        self.assertEqual(self.device.registers[4][40],1)

    def test_alarm_checksum_and_motion_monitor_gap_still_immediate_faults(self):
        for failure in ('alarm','checksum','gap'):
            with self.subTest(failure=failure):
                if self.bridge:self.bridge.close()
                self.device=SerialMock(self.clock); b=self.policy(running=True)
                self.device.registers[4][63]=150
                if failure=='alarm': self.device.registers[4][65]=4
                elif failure=='checksum': self.device.alter=lambda x:x[:-1]+bytes((x[-1]^1,))
                else: self.clock.sleep(.31)
                with self.assertRaises(CalibrationError):b.keep()
                self.assert_off(); self.assertFalse(b.committed)

    def test_calibrator_does_not_apply_old_55_threshold_after_transport_validation(self):
        b=self.policy(running=True); self.device.registers[4][63]=150
        runner=Calibrator(b,self.folder)
        feedback=runner.feedback(4)
        self.assertEqual(feedback['temperature'],150)
        self.assertTrue(b.active)

    def test_commit_requires_end_of_pending_high_episode(self):
        b=self.policy(); self.device.registers[3][63]=150
        b.read(3,63,1)
        calls=[]
        def validate_commit():
            self.assertFalse(b.temperature_guard.pending); calls.append(True)
        with patch.object(b,'_commit_validated',validate_commit):
            with self.assertRaises(CalibrationError):b.finish(True)
        self.assertFalse(calls or b.committed); self.assert_off()

    def test_canonical_backup_is_initial_healthy_snapshot(self):
        b=self.policy(); self.device.registers[3][63]=150
        runner=Calibrator(b,self.folder)
        with contextlib.redirect_stdout(io.StringIO()):data=runner.backup()
        self.assertTrue(all(x['temperature']<=45 for x in data['motors']))
        self.assertEqual(runner.original[3]['ram_hex'],b.original[3][40:].hex())

    def test_mechanical_range_policy_does_not_mistake_old_soft_limit_for_hard_boundary(self):
        b=self.policy(); runner=Calibrator(b,self.folder,discover_mechanical_range=True)
        runner.original[3]={'range_min':1000,'range_max':2000}
        runner.tracks[3]={'last':2000,'unwrapped':2000,'old_pos':2000,'raw_start':2000}
        self.device.physical[3]=2200
        self.assertEqual(runner.feedback(3)['position'],2200)
        self.assertFalse(runner.endpoints or runner.results)  # not a measured endpoint
        runner.discover_mechanical_range=False
        with self.assertRaises(CalibrationError):runner.feedback(3)

    def test_discovery_outside_old_soft_range_holds_current_pose_after_off_limit_setup(self):
        self.device.physical[4]=3500
        self.bridge=usb.Bridge('MOCK',self.folder,lambda *a,**k:self.device,
                               temperature_debounce=True,discover_mechanical_range=True)
        self.bridge.start(); b=self.bridge
        runner=Calibrator(b,self.folder,discover_mechanical_range=True)
        with contextlib.redirect_stdout(io.StringIO()):runner.backup();runner.prepare_holds()
        self.assertEqual(int.from_bytes(self.device.registers[4][42:44],'little'),3500)
        self.assertEqual(int.from_bytes(self.device.registers[4][9:11],'little'),0)
        self.assertEqual(int.from_bytes(self.device.registers[4][11:13],'little'),4095)
        b.finish(False);self.assert_off()
        for motor in range(1,7):self.assertEqual(bytes(self.device.registers[motor][:40]),b.original[motor][:40])


if __name__=='__main__':unittest.main()
