from pathlib import Path
import sys,unittest,time,tempfile
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from button_input import ButtonPose,ButtonJoyCon
import right_joycon as h
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_trial_host import UART,Joy

class Thread:
    def is_alive(self):return True
    def join(self,timeout):pass
class Device:
    def __init__(self):self._update_input_report_thread=Thread();self.values={};self.closed=False
    def register_update_hook(self,f):self.hook=f;f(self)
    def __getattr__(self,n):
        if n.startswith('get_button_'):return lambda:self.values.get(n[11:],False)
        raise AssertionError('Unexpected non-button device access: '+n)
    def _close(self):self.closed=True

class Tests(unittest.TestCase):
    def test_no_enable_no_motion(self):
        p=ButtonPose();self.assertEqual(p.update({'x':True},.2)[0],[0]*6)
    def test_translation_and_release(self):
        p=ButtonPose();p.update({'zr':True,'x':True},.2)
        self.assertAlmostEqual(p.pose[2],.001)
        before=list(p.pose);self.assertEqual(p.update({},.2)[0],before)
    def test_opposing_buttons_cancel(self):
        p=ButtonPose();p.update({'zr':True,'x':True,'b':True,'a':True,'y':True},.2)
        self.assertEqual(p.pose,[0]*6)
    def test_rotation_mode_is_only_roll_pitch(self):
        p=ButtonPose();p.update({'zr':True,'r':True,'x':True,'a':True,'right_sr':True},.2)
        self.assertEqual(p.pose[:3],[0]*3);self.assertAlmostEqual(p.pose[3],.02);self.assertAlmostEqual(p.pose[4],.02);self.assertEqual(p.pose[5],0)
    def test_home_and_plus(self):
        p=ButtonPose();self.assertTrue(p.update({'home':True},.2)[3])
        with self.assertRaises(KeyboardInterrupt):p.update({'plus':True},.2)
    def test_elapsed_is_bounded(self):
        p=ButtonPose();p.update({'zr':True,'right_sr':True},100)
        self.assertAlmostEqual(p.pose[0],.001)
    def test_device_uses_buttons_only_and_stale_reports_stop(self):
        d=Device()
        import types
        mods={'joyconrobotics':types.ModuleType('joyconrobotics'),'joyconrobotics.joycon':types.SimpleNamespace(JoyCon=lambda *a:d),'joyconrobotics.device':types.SimpleNamespace(get_R_id=lambda:(1,2,'fake'))}
        with patch.dict(sys.modules,mods):source=ButtonJoyCon()
        d.values={'zr':True,'x':True};d.hook(d);source.previous=time.monotonic()-.2
        self.assertGreater(source.read(0)[0][2],0)
        source.last_report=time.monotonic()-1
        with self.assertRaisesRegex(RuntimeError,'stale'):source.read(0)
        source.close();self.assertTrue(d.closed)
    def test_buttons_cli_and_cleanup(self):
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',UART),patch('button_input.ButtonJoyCon',Joy),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','buttons','--port','FAKE','--test-drive','--manual-control','--count','2','--csv',str(Path(d)/'buttons.csv')])
        self.assertEqual(UART.last.seq,0);self.assertEqual(UART.last.calls.count('ARM1 PAUSE'),0);self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
    def test_pressed_buttons_solve_and_release_holds(self):
        class PressJoy(Joy):
            n=0
            def read(self,t):
                self.n+=1
                return [0]*6,.5,(1<<7)|1 if self.n==3 else 0,False
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',UART),patch('button_input.ButtonJoyCon',PressJoy),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','buttons','--port','FAKE','--test-drive','--manual-control','--count','2','--csv',str(Path(d)/'buttons.csv')])
        self.assertEqual(UART.last.seq,1);self.assertNotIn('ARM1 PAUSE',UART.last.calls);self.assertEqual(UART.last.calls[-1],'ARM1 STOP')

if __name__=='__main__':unittest.main()
