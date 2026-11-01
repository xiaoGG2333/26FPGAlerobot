"""Offline rejection recovery tests. No physical serial ports or motors."""
import contextlib,csv,io,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'host'),str(ROOT/'tests')]
import right_joycon as h
from test_trial_host import UART,Joy,Q

class RecoveryTests(unittest.TestCase):
 def run_session(self,code,manual=False,verified=False,stopped=False):
  class Bus(UART):
   def __init__(self,*a):super().__init__(*a);self.history=[]
   def exchange(self,*a):return super().exchange(*a)+' POSITION_ROLL_IK_20261019_V1 COORDINATED_MANUAL_V1'
   def arm(self,c,e):
    self.history.append(c)
    if c=='ARM1 CHECK':self.calls.append(c);return 'ARM1 CHECK_OK OK active=0'
    if c=='ARM1 ENABLE':self.calls.append(c);return 'ARM1 READY OK'
    if c=='ARM1 STATE':
     self.calls.append(c)
     flags='mode=2 aligned=1 geometry=1' if verified else 'mode=3 aligned=0 geometry=0'
     return 'ARM1 STATE active=%d %s result=OK torque_off=%d counts=2149,2050,2017,1985,1023 goal=2149,2050,2017,1985,1023'%((0 if stopped and self.seq==2 else 1),flags,(1 if stopped and self.seq==2 else 0))
    return super().arm(c,e)
   def solve(self,*a):
    self.seq+=1;self.history.append('SOLVE%d'%self.seq)
    assert 'ARM1 STOP' not in self.history
    v=code if self.seq==2 else 0
    return dict(seq=self.seq,code=v,status=h.CODES[v],iterations=200 if v else 1,position_error=.001 if v else 1e-5,rotation_error=0,q=Q,gripper=.5,roundtrip_ms=30)
   def counts(self):return [2149,2050,2017,1985,1023,2773],int(verified)
  UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
  with tempfile.TemporaryDirectory(dir=ROOT/'reports') as d:
   args=['--source','joycon','--port','FAKE','--count','3','--csv',str(Path(d)/'out.csv')]
   args+=['--drive'] if verified else ['--test-drive']
   if manual:args+=['--manual-control']
   with patch.object(h,'UART',Bus),patch.object(h,'RightJoyCon',Joy),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'),patch.object(h,'rollback_input',wraps=h.rollback_input) as rollback,contextlib.redirect_stdout(io.StringIO()):
    if stopped:
     with self.assertRaisesRegex(RuntimeError,'Arm stopped'):h.main(args)
     return
    h.main(args)
   with (Path(d)/'out.csv').open() as stream:rows=list(csv.DictReader(stream))
   self.assertEqual([row['status'] for row in rows],['CONVERGED',h.CODES[code],'CONVERGED'])
   self.assertEqual(rows[1]['count_status'],'REJECTED_KEEP_LAST_GOAL')
   self.assertEqual(rollback.call_count,1)
   self.assertEqual(UART.last.seq,3)
   self.assertEqual(UART.last.history.count('ARM1 STOP'),1)
   self.assertEqual(UART.last.history[-1],'ARM1 STOP') # normal program exit still stops
 def test_manual_all_solver_error_codes_continue(self):
  for code in range(1,5):
   with self.subTest(code=code):self.run_session(code,manual=True)
 def test_nonmanual_trial_failure_continues(self):self.run_session(1)
 def test_verified_drive_failure_continues(self):self.run_session(4,verified=True)
 def test_actual_board_stop_remains_terminal(self):self.run_session(1,manual=True,stopped=True)
if __name__=='__main__':unittest.main()

