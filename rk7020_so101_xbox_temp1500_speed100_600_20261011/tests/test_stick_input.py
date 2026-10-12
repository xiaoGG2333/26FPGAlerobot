from pathlib import Path
import sys,time,tempfile,unittest,types
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from stick_input import StickPose,StickJoyCon
import right_joycon as h
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_trial_host import UART,Joy
from test_button_input import Thread

class Device:
    def __init__(self):self.h=2500;self.v=2500;self.r=False;self.press=False;self.closed=False;self._update_input_report_thread=Thread()
    def get_stick_right_horizontal(self):return self.h
    def get_stick_right_vertical(self):return self.v
    def get_button_r(self):return self.r
    def get_button_r_stick(self):return self.press
    def register_update_hook(self,f):self.hook=f;f(self)
    def __getattr__(self,n):raise AssertionError('Forbidden input access '+n)
    def _close(self):self.closed=True

class Tests(unittest.TestCase):
    def test_xy_direction_matches_upstream(self):
        for raw_h,raw_v,axis,sign in [(2500,4095,0,1),(2500,0,0,-1),(4095,2500,1,-1),(0,2500,1,1)]:
            with self.subTest(h=raw_h,v=raw_v):
                p=StickPose();pose,_,mask,_=p.update(raw_h,raw_v,False,False,.2)
                self.assertAlmostEqual(pose[axis],sign*.001);self.assertNotEqual(mask,0);self.assertEqual(pose[3:],[0]*3)
    def test_press_down_r_up_and_cancel(self):
        for press,r,z in [(True,False,-.001),(False,True,.001),(True,True,0)]:
            p=StickPose();pose,_,mask,_=p.update(2500,2500,press,r,.2);self.assertAlmostEqual(pose[2],z);self.assertEqual(bool(mask),bool(z))
    def test_neutral_inclusive_thresholds(self):
        p=StickPose()
        for h,v in [(1000,4000),(4000,1000),(2500,2500)]:
            self.assertEqual(p.update(h,v,False,False,.2)[2],0)
        self.assertEqual(p.pose,[0]*6)
    def test_timing_bound_and_invalid_data(self):
        p=StickPose();self.assertAlmostEqual(p.update(2500,4095,False,False,100)[0][0],.001)
        with self.assertRaises(ValueError):p.update(-1,2500,False,False,.2)
    def test_only_stick_and_r_are_read_and_stale_stops(self):
        d=Device();mods={'joyconrobotics':types.ModuleType('joyconrobotics'),'joyconrobotics.joycon':types.SimpleNamespace(JoyCon=lambda *a:d),'joyconrobotics.device':types.SimpleNamespace(get_R_id=lambda:(1,2,'fake'))}
        with patch.dict(sys.modules,mods):s=StickJoyCon()
        self.assertEqual(s.read(0)[2],0)
        d.h=0;d.hook(d);s.previous=time.monotonic()-.2
        self.assertGreater(s.read(0)[0][1],0)
        s.last_report=time.monotonic()-1
        with self.assertRaisesRegex(RuntimeError,'stale'):s.read(0)
        s.close();self.assertTrue(d.closed)
    def test_idle_no_pose_packets(self):
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',UART),patch('stick_input.StickJoyCon',Joy),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','stick','--port','FAKE','--test-drive','--manual-control','--count','2','--csv',str(Path(d)/'idle.csv')])
        self.assertEqual(UART.last.seq,0);self.assertEqual(UART.last.calls.count('ARM1 PAUSE'),0);self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
    def test_stick_motion_then_release(self):
        class Stick(Joy):
            n=0
            def read(self,t):
                self.n+=1;return [0]*6,.5,1 if self.n==3 else 0,False
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',UART),patch('stick_input.StickJoyCon',Stick),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','stick','--port','FAKE','--test-drive','--manual-control','--count','2','--csv',str(Path(d)/'move.csv')])
        self.assertEqual(UART.last.seq,1);self.assertNotIn('ARM1 PAUSE',UART.last.calls);self.assertEqual(UART.last.calls[-1],'ARM1 STOP')

if __name__=='__main__':unittest.main()
