"""Host-only regression using fake UART/controller; never opens a real port."""
from pathlib import Path
import contextlib,io,sys,tempfile,unittest
from unittest.mock import patch
r=Path(__file__).resolve().parents[1];sys.path[:0]=[str(r/'host'),str(r/'tests')]
import right_joycon as h
import test_trial_host as fixture
exchange=fixture.UART.exchange
fixture.UART.exchange=lambda self,*args:exchange(self,*args)+' POSITION_ROLL_IK_20261019_V1'
class PositionTaskTests(unittest.TestCase):
    def test_id5_reference_is_joint_angle_not_euler_branch(self):
        u=h.UART.__new__(h.UART);u.seq=0
        u.exchange=lambda *args:'STATE1 0 0 .5 .6 .7 .2 .2 0 .2 -2.94159265 1.34 0 0'
        q,pose=u.state();self.assertEqual(pose[3],q[4]);self.assertEqual(pose[3],.2)
    def test_old_firmware_rejected_before_enable(self):
        class Old(fixture.UART):
            def exchange(self,*args):return exchange(self,*args)
        with patch.object(h,'UART',Old):
            with self.assertRaisesRegex(RuntimeError,'Position-priority host requires'):
                h.main(['--source','xbox','--port','FAKE','--test-drive','--manual-control'])
        self.assertEqual(fixture.UART.last.calls,[]);self.assertTrue(fixture.UART.last.closed)
suite=unittest.defaultTestLoader.loadTestsFromNames(['test_xbox_input','test_finish_target','test_hold_host','test_chip_telemetry'])
suite.addTests(unittest.defaultTestLoader.loadTestsFromTestCase(PositionTaskTests))
stream=io.StringIO()
with tempfile.TemporaryDirectory(dir=r/'reports') as tmp:
    tempfile.tempdir=tmp
    with contextlib.redirect_stdout(io.StringIO()):result=unittest.TextTestRunner(stream=stream).run(suite)
print(stream.getvalue());(r/'reports/position_roll_host_tests.txt').write_text(stream.getvalue())
raise SystemExit(0 if result.wasSuccessful() else 1)
