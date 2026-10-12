"""Host trial handshake/cleanup checks with no serial hardware."""
from pathlib import Path
import sys,math,tempfile,csv,unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
import right_joycon as h
Q=[math.radians(v) for v in [-5.44921875,-17.9296875,26.982421875,-5.361328125,90.615234375]]
class Joy:
    def read(self,t):return [0]*6,.5,0,False
    def close(self):pass
class UART:
    last=None;wrong=False;failed=False;lost_enable=False;limit=False
    def __init__(self,*args):self.calls=[];self.closed=False;self.seq=0;UART.last=self
    def exchange(self,*args):return 'POSE1 READY SO101_135MM ARM_CONTROL_V1 MANUAL_REFERENCE_20261009_V1 SYNC_TARGET_HOLD_V1 LIMITS600_TEMP1500_20261011_V1'+('' if UART.wrong else ' LIMITED_DRIVE_V1 MANUAL_OUT_HOLD_V2 BUTTONS_INPUT_V1 STICK_INPUT_V1')
    def state(self,*args,**kwargs):return Q,h.fk_pose(Q)
    def arm(self,c,e):
        self.calls.append(c)
        if c in ('ARM1 TEST_CHECK','ARM1 TEST_MANUAL_CHECK'):return 'ARM1 CHECK_OK OK active=0'
        if c in ('ARM1 TEST_ENABLE','ARM1 TEST_MANUAL_ENABLE'):
            if UART.lost_enable:raise TimeoutError('enable acknowledgement lost')
            return 'ARM1 READY LIMITED_DRIVE_V1'
        if c=='ARM1 KEEP':return 'ARM1 ALIVE OK active=1'
        if c=='ARM1 PAUSE':return 'ARM1 HELD OK active=1 torque_off=0'
        if c=='ARM1 STOP':return 'ARM1 STOP STOPPED active=0 torque_off=1'
        if c=='ARM1 STATE':return 'ARM1 STATE active=1 mode=3 aligned=0 geometry=0 result=OK torque_off=0 counts=2149,2050,2017,1985,1023 goal=2149,2050,2017,1985,1023'
        raise AssertionError(c)
    def solve(self,core,target,gripper):
        self.seq+=1
        return dict(seq=self.seq,code=2 if UART.failed else 4 if UART.limit else 0,status='HARDWARE_ERROR' if UART.failed else 'JOINT_LIMIT' if UART.limit else 'CONVERGED',iterations=1,position_error=1e-5,rotation_error=1e-6,q=Q,gripper=gripper,roundtrip_ms=25.)
    def counts(self):return [2149,2050,2017,1985,1023,2773],0
    def close(self):self.closed=True
class Tests(unittest.TestCase):
    def setUp(self):UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
    def run_host(self,out):
        args=['right_joycon.py','--source','joycon','--port','FAKE','--test-drive','--count','2','--csv',str(out)]
        with patch.object(h,'UART',UART),patch.object(h,'RightJoyCon',Joy),patch.object(sys,'argv',args),patch.object(h.time,'sleep'):
            h.main()
    def test_unverified_trial_is_labelled_and_stopped(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'trial.csv';self.run_host(out)
            with out.open(encoding='utf8') as stream:rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),2)
            self.assertTrue(all(r['count_status']=='LIMITED_TRIAL_UNVERIFIED' and r['physical_alignment_verified']=='0' for r in rows))
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP');self.assertTrue(UART.last.closed)
        self.assertNotIn('ARM1 ENABLE',UART.last.calls)
    def test_solver_failure_stops(self):
        UART.failed=True
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError,'IK failed'):
                self.run_host(Path(d)/'out.csv')
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP');self.assertTrue(UART.last.closed)
    def test_previous_firmware_rejected_without_enable(self):
        UART.wrong=True
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError,'trial BOOT'):
                self.run_host(Path(d)/'out.csv')
        self.assertEqual(UART.last.calls,[]);self.assertTrue(UART.last.closed)
    def test_enable_ack_loss_also_stops(self):
        UART.lost_enable=True
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaises(TimeoutError):self.run_host(Path(d)/'out.csv')
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP');self.assertTrue(UART.last.closed)
    def test_manual_out_continues_until_count_end(self):
        UART.limit=True
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',UART),patch.object(h,'RightJoyCon',Joy),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','joycon','--port','FAKE','--test-drive','--manual-control','--count','2','--csv',str(Path(d)/'out.csv')])
        self.assertEqual(UART.last.seq,2)
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
    def test_manual_cancel_does_not_enable(self):
        with patch.object(h,'UART',UART),patch.object(h,'RightJoyCon',Joy),patch('builtins.input',side_effect=KeyboardInterrupt):
            with self.assertRaises(KeyboardInterrupt):
                h.main(['--source','joycon','--port','FAKE','--test-drive','--manual-control'])
        self.assertNotIn('ARM1 TEST_MANUAL_ENABLE',UART.last.calls)
        self.assertTrue(UART.last.closed)
    def test_manual_ctrl_c_confirms_stop(self):
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',UART),patch.object(h,'RightJoyCon',Joy),patch('builtins.input',return_value=''),patch.object(UART,'solve',side_effect=KeyboardInterrupt):
                with self.assertRaises(KeyboardInterrupt):
                    h.main(['--source','joycon','--port','FAKE','--test-drive','--manual-control','--csv',str(Path(d)/'manual.csv')])
        self.assertIn('ARM1 TEST_MANUAL_ENABLE',UART.last.calls)
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP');self.assertTrue(UART.last.closed)
if __name__=='__main__':unittest.main()
