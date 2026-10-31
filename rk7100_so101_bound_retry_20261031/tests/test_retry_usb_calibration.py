"""Retry orchestration tests: no serial ports, no physical hardware."""
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import retry_usb_calibration as retry
from test_auto_calibrate_usb import Clock


class RetryTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.calls = []
        self.clock = Clock()
        self.patches = [patch.object(retry.time, 'monotonic', self.clock.monotonic),
                        patch.object(retry.time, 'sleep', self.clock.sleep)]
        for p in self.patches: p.start()
    def tearDown(self):
        for p in reversed(self.patches): p.stop()
        self.temp.cleanup()
    def execute(self, argv, axis=None, stop=False, bad=False):
        folder = Path(argv[argv.index('--out') + 1]); folder.mkdir()
        self.calls.append(argv)
        report = {'status': 'CALIBRATION_PASS', 'real_calibration_complete': True, 'commit_confirmed': True}
        if axis:
            report = {'status': 'NOT_COMPLETED', 'commit_confirmed': False,
                      'cleanup': ['USB STOP off_confirmed=1 rollback_success=1'],
                      'temperature_recheck': {'id': axis, 'classification': 'HIGH_NOT_REPRODUCED_AFTER_CONFIRMED_STOP',
                                              'cleanup_confirmed': True, 'samples': [{}] * 5,
                                              'recheck_duration_s': 2.1}}
        if bad: report['transport_log_error'] = 'disk full'
        if stop: (self.folder / 'STOP').write_text('stop')
        (folder / 'summary.json').write_text(json.dumps(report))
        if axis: raise retry.usb.TemperatureNotReproduced('mock high')
    def run_mock(self, execute, attempts=3):
        with contextlib.redirect_stdout(io.StringIO()):
            return retry.run_attempts(self.folder, attempts=attempts, execute=execute)
    def test_transient_then_new_folder_success_and_shared_stop(self):
        def execute(argv): self.execute(argv, axis=3 if not self.calls else None)
        report = self.run_mock(execute)
        self.assertTrue(report['real_calibration_complete'])
        self.assertEqual(len(self.calls), 2)
        self.assertNotEqual(self.calls[0][self.calls[0].index('--out')+1], self.calls[1][self.calls[1].index('--out')+1])
        for argv in self.calls:
            self.assertEqual(Path(argv[argv.index('--stop-file')+1]), self.folder / 'STOP')
        self.assertFalse(report['old_endpoint_reuse'] or report['initial_high_readings_proven_false'])
    def test_same_axis_second_high_aborts_with_two_sessions(self):
        with self.assertRaises(retry.CalibrationError): self.run_mock(lambda a: self.execute(a, axis=3))
        self.assertEqual(len(self.calls), 2)
    def test_different_axes_still_cannot_exceed_three_sessions(self):
        with self.assertRaises(retry.CalibrationError):
            self.run_mock(lambda a: self.execute(a, axis=len(self.calls)+1))
        self.assertEqual(len(self.calls), 3)
    def test_parent_stop_or_log_error_blocks_retry(self):
        with self.assertRaises(retry.CalibrationError): self.run_mock(lambda a: self.execute(a, axis=3, stop=True))
        self.assertEqual(len(self.calls), 1)
    def test_unrelated_or_keyboard_error_never_retries(self):
        def fail(argv):
            self.calls.append(argv); raise KeyboardInterrupt()
        with self.assertRaises(KeyboardInterrupt): self.run_mock(fail)
        self.assertEqual(len(self.calls), 1)
    def test_log_fault_in_child_blocks_retry(self):
        with self.assertRaises(retry.CalibrationError): self.run_mock(lambda a: self.execute(a, axis=3, bad=True))
        self.assertEqual(len(self.calls), 1)

    def test_committed_output_failure_is_preserved_without_retry(self):
        def execute(argv):
            self.execute(argv)
            folder = Path(argv[argv.index('--out')+1])
            (folder/'summary.json').write_text(json.dumps({'status':'COMMITTED_OUTPUT_ERROR', 'commit_confirmed':True}))
            raise OSError('Final JSON write failed after COMMIT')
        with self.assertRaises(OSError): self.run_mock(execute)
        report = json.loads((self.folder/'summary.json').read_text())
        self.assertTrue(report['commit_confirmed'])
        self.assertEqual(report['status'], 'COMMITTED_OUTPUT_ERROR')
        self.assertEqual(len(self.calls), 1)

    def test_close_error_after_transient_does_not_retry(self):
        def execute(argv):
            try: self.execute(argv, axis=3)
            finally:
                folder=Path(argv[argv.index('--out')+1])
                child=json.loads((folder/'summary.json').read_text())
                child['close_error']='Serial close failed'
                (folder/'summary.json').write_text(json.dumps(child))
        with self.assertRaises(retry.CalibrationError): self.run_mock(execute)
        self.assertEqual(len(self.calls),1)

    def test_close_error_after_commit_preserves_committed_state(self):
        def execute(argv):
            self.execute(argv)
            folder=Path(argv[argv.index('--out')+1])
            child=json.loads((folder/'summary.json').read_text())
            child['close_error']='Serial close failed'
            (folder/'summary.json').write_text(json.dumps(child))
        with self.assertRaises(retry.CalibrationError): self.run_mock(execute)
        report=json.loads((self.folder/'summary.json').read_text())
        self.assertEqual(report['status'],'COMMITTED_OUTPUT_ERROR')
        self.assertTrue(report['commit_confirmed'])
        self.assertEqual(len(self.calls),1)


if __name__ == '__main__': unittest.main()
