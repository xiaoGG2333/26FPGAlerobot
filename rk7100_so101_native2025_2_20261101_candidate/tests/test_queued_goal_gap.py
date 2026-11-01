from pathlib import Path
import sys,tempfile,unittest,csv
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_trial_host import UART,Joy
import right_joycon as h

class MovingInput(Joy):
    def read(self,t):return [0]*6,.5,16,False

class QueuedUART(UART):
    coordinated=True
    stopped=False
    def exchange(self,*args):
        return super().exchange(*args)+(' COORDINATED_MANUAL_V1' if self.coordinated else '')
    def counts(self):return [2149,2006,2017,1985,1023,2773],0
    def arm(self,command,expected):
        if self.stopped and command=='ARM1 KEEP':
            self.calls.append(command)
            return 'ARM1 BLOCKED NOT_ENABLED active=0 torque_off=1'
        return super().arm(command,expected)

class Tests(unittest.TestCase):
    def setUp(self):
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        QueuedUART.coordinated=True;QueuedUART.stopped=False
    def run_host(self,path):
        with patch.object(h,'UART',QueuedUART),patch.object(h,'RightJoyCon',MovingInput),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
            h.main(['--source','joycon','--port','FAKE','--test-drive','--manual-control','--count','3','--csv',str(path)])
    def test_final_goal_gap_does_not_stop_coordinated_manual_ramp(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'results.csv';self.run_host(p)
            with p.open() as f:rows=list(csv.DictReader(f))
        self.assertEqual(UART.last.seq,3)
        self.assertTrue(all(r['status']=='CONVERGED' for r in rows))
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
    def test_legacy_noncoordinated_follow_check_retained(self):
        QueuedUART.coordinated=False
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError,'Joint following error'):
                self.run_host(Path(d)/'results.csv')
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
    def test_board_fault_still_stops_host(self):
        QueuedUART.stopped=True
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError,'Arm session stopped'):
                self.run_host(Path(d)/'results.csv')
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP')

if __name__=='__main__':unittest.main()
