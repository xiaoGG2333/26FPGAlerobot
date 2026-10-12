"""No hardware: isolated-axis addressing and read-only diagnostic boundaries."""
from contextlib import redirect_stdout
import io, json
from pathlib import Path
import sys, tempfile, unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'host'))
import diagnose_usb_feedback as diagnostic


class FakeBus:
    instances = []
    temperature = 31

    def __init__(self, *_):
        self.log_error = None
        self.calls = []
        self.closed = False
        self.instances.append(self)

    def read(self, motor, address, count):
        self.calls.append((motor, address, count))
        memory = bytearray(80)
        memory[3:5] = (777).to_bytes(2, 'little')
        memory[5] = motor
        memory[9:11] = (900).to_bytes(2, 'little')
        memory[11:13] = (3100).to_bytes(2, 'little')
        memory[56:58] = (2000).to_bytes(2, 'little')
        memory[62] = 122
        memory[63] = self.temperature
        return bytes(memory[address:address+count])

    def close(self):
        self.closed = True


class Tests(unittest.TestCase):
    def run_diagnostic(self, isolated=False, hot=False):
        FakeBus.instances = []
        FakeBus.temperature = 77 if hot else 31
        with tempfile.TemporaryDirectory() as temporary:
            folder = Path(temporary) / 'out'
            arguments = ['diagnostic', '--motor', '3', '--rounds', '1', '--out', str(folder)]
            if isolated:
                arguments.append('--isolated')
            with patch.object(sys, 'argv', arguments), patch.object(diagnostic, 'USBProtocol', FakeBus), \
                    patch.object(diagnostic.time, 'sleep', lambda _: None), redirect_stdout(io.StringIO()):
                if hot:
                    with self.assertRaises(diagnostic.CalibrationError):
                        diagnostic.main()
                else:
                    diagnostic.main()
            return json.loads((folder / 'summary.json').read_text()), FakeBus.instances[-1]

    def test_isolated_id3_never_addresses_disconnected_axes(self):
        result, bus = self.run_diagnostic(isolated=True)
        self.assertEqual({call[0] for call in bus.calls}, {3})
        self.assertEqual(result['status'], 'READ_ONLY_CHECK_COMPLETE')
        self.assertEqual(result['checked_motors'], [3])
        self.assertTrue(result['after'][0]['eeprom_matches_before'])
        self.assertTrue(bus.closed)

    def test_default_checks_all_six_without_a_write_interface(self):
        result, bus = self.run_diagnostic()
        self.assertEqual({call[0] for call in bus.calls}, set(range(1, 7)))
        self.assertEqual(len(result['after']), 6)
        self.assertFalse(result['motion_commands_sent'])
        self.assertEqual(len(result['samples']), 10)

    def test_hot_read_aborts_and_closes_without_motion(self):
        result, bus = self.run_diagnostic(isolated=True, hot=True)
        self.assertEqual(result['status'], 'NOT_COMPLETED')
        self.assertFalse(result['motion_commands_sent'])
        self.assertTrue(bus.closed)


if __name__ == '__main__':
    unittest.main()
