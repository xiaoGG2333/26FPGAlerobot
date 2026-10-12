from pathlib import Path
import csv,json,math,sys,tempfile,unittest
from types import SimpleNamespace
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'));sys.path.insert(0,str(ROOT/'tests'))
from xbox_input import XboxPose,XboxController,axis_value,pick_axes,devices
from test_trial_host import UART
import right_joycon as h

class FakeDevice:
    name='Xbox Wireless Controller';path='/dev/input/event9'
    def __init__(self):self.values={0:0,1:0,3:0,4:0};self.closed=False;self.removed=False
    def capabilities(self,absinfo=True):
        return {3:[(code,self.absinfo(code)) for code in self.values] if absinfo else list(self.values)}
    def absinfo(self,axis):
        if self.removed:raise OSError('Device disconnected')
        return SimpleNamespace(value=self.values[axis],min=-32768,max=32767)
    def read(self):raise BlockingIOError()
    def close(self):self.closed=True

class Tests(unittest.TestCase):
    def test_microsoft_controller_name_discovery(self):
        d=FakeDevice();d.name='Microsoft Controller';d.info=SimpleNamespace(vendor=0x045e)
        d.capabilities=lambda **kw:{3:[0,1,3,4],1:[304]}
        with patch.dict(sys.modules,{'evdev':SimpleNamespace(InputDevice=lambda path:d,list_devices=lambda:[d.path])}):
            self.assertEqual(devices(),[d])
        self.assertFalse(d.closed);d.close()
    def test_series_x_axes_with_trigger_capabilities(self):
        d=FakeDevice();d.name='Microsoft Xbox Series S|X Controller'
        d.values={0:361,1:1182,2:0,3:1298,4:786,5:0,16:0,17:0}
        self.assertEqual(pick_axes(d),( (0,1,3,4),'rxry'))
        # Trigger zeros must not appear as fully deflected stick values.
        with patch.dict(sys.modules,{'evdev':SimpleNamespace(InputDevice=lambda path:d)}):
            c=XboxController(d.path)
            self.assertEqual(c.report,(0.,0.,0.,0.));c.close()
    def test_all_requested_directions(self):
        cases=[((0,-1,0,0),0,1),((0,1,0,0),0,-1),
               ((-1,0,0,0),1,-1),((1,0,0,0),1,1),
               ((0,0,0,-1),2,1),((0,0,0,1),2,-1),
               ((0,0,-1,0),3,-1),((0,0,1,0),3,1)]
        for axes,index,sign in cases:
            pose,gripper,mask,home=XboxPose().update(*axes,.2)
            self.assertEqual(sum(v!=0 for v in pose),1)
            self.assertGreater(pose[index]*sign,0)
            self.assertTrue(mask);self.assertEqual(gripper,.5);self.assertFalse(home)
    def test_centre_and_deadzone_hold_pose(self):
        for value in (-1000,0,1000):self.assertEqual(axis_value(value,-32768,32767),0)
        p=XboxPose();first=p.update(1,0,0,0,.2)[0]
        pose,_,mask,_=p.update(0,0,0,0,.2)
        self.assertEqual(pose,first);self.assertEqual(mask,0)
    def test_diagonal_speed_and_delay_are_bounded(self):
        pose=XboxPose().update(1,-1,1,-1,20)[0]
        self.assertAlmostEqual(math.sqrt(sum(v*v for v in pose[:3])),.004)
        self.assertAlmostEqual(pose[3],.02)
        a=XboxPose().update(.5,0,0,0,.2)[0]
        self.assertAlmostEqual(a[1],.002)
        for v in (float('nan'),2):
            with self.assertRaises(ValueError):XboxPose().update(v,0,0,0,.2)
    def test_right_roll_means_ccw_and_decreasing_counts(self):
        q=[0,-.2,.3,.1,.2];pose=h.fk_pose(q)
        c=json.loads((ROOT/'host/right_config.json').read_text())
        mapper=h.Mapper(c,pose,[0]*6)
        target=mapper.target(XboxPose().update(0,0,1,0,.2)[0],.2)
        self.assertAlmostEqual(h.wrap(target[3]-pose[3]),.02)
        moved=list(q);moved[4]+=.02
        self.assertAlmostEqual(h.wrap(h.fk_pose(moved)[3]-pose[3]),.02)
        m=json.loads((ROOT/'host/right_mapping.json').read_text())['joint_mapping']['wrist_roll']
        self.assertEqual(m['sign'],-1)
        self.assertLess(m['sign']*.02*m['counts_per_rad'],0)
    def test_live_state_without_repeated_events_and_disconnect(self):
        d=FakeDevice()
        with patch.dict(sys.modules,{'evdev':SimpleNamespace(InputDevice=lambda path:d)}):
            c=XboxController('/dev/input/event9')
            d.values[1]=-32768;c.previous-=.2
            self.assertGreater(c.read(0)[0][0],0)
            d.removed=True
            with self.assertRaises(OSError):c.read(0)
            c.close();self.assertTrue(d.closed)
    def test_axis_mode_and_startup_neutral(self):
        d=FakeDevice();self.assertEqual(pick_axes(d)[0],(0,1,3,4))
        d.values={0:0,1:0,2:0,5:0};self.assertEqual(pick_axes(d)[0],(0,1,2,5))
        d.values[0]=32767
        with patch.dict(sys.modules,{'evdev':SimpleNamespace(InputDevice=lambda path:d)}):
            with self.assertRaisesRegex(RuntimeError,'Centre both'):XboxController(d.path)
        self.assertTrue(d.closed)
    def test_xbox_motion_then_neutral_preserves_goal_and_stops_on_exit(self):
        class Source:
            def __init__(self,*a):self.n=0;self.report=(0,0,0,0);self.closed=False
            def read(self,*a):
                self.n+=1
                return ([.004,0,0,0,0,0],.5,1,False) if self.n==3 else ([.004,0,0,0,0,0],.5,0,False)
            def close(self):self.closed=True
        class Bus(UART):
            instance=None
            def __init__(self,*a):super().__init__(*a);Bus.instance=self
            def exchange(self,*a):return super().exchange(*a)+' COORDINATED_MANUAL_V1'
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder)/'xbox.csv'
            with patch('xbox_input.XboxController',Source),patch.object(h,'UART',Bus),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','xbox','--port','FAKE','--test-drive','--manual-control','--count','2','--csv',str(out)])
            self.assertNotIn('ARM1 PAUSE',Bus.instance.calls)
            self.assertEqual(Bus.instance.calls[-1],'ARM1 STOP')
            with out.open() as f:rows=list(csv.DictReader(f))
            self.assertIn('xbox_lx',rows[0]);self.assertTrue(rows[1]['status'].startswith('XBOX_'))

if __name__=='__main__':unittest.main()
