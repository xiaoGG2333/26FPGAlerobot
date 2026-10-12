import unittest,sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from right_joycon import Mapper,rollback_input,UART
class Tests(unittest.TestCase):
 def test_rollback_retains_valid_goal_and_discards_input(self):
  config={'position_scale':1,'axis_matrix':[[1,0,0],[0,1,0],[0,0,1]],'workspace_min':[-1]*3,'workspace_max':[1]*3,'max_translation_m_s':.02,'rotation_scale':1,'max_rotation_rad_s':.1}
  source=SimpleNamespace(integrator=SimpleNamespace(pose=[0,0,.3,0,0,0]),previous=0)
  m=Mapper(config,[.2,0,.1,0,0,0],[0]*6);m.commit([.2,0,.15,0,0,0])
  m=rollback_input(source,m,[0,0,.3,0,0,0])
  self.assertEqual(source.integrator.pose,[0]*6)
  self.assertEqual(m.target([0,0,0,0,0,0],.2)[:3],[.2,0,.15])
  self.assertLess(m.target([0,0,-.004,0,0,0],.2)[2],.15)
 def test_frozen_event_is_recoverable(self):
  u=UART.__new__(UART);u.motion_expected=True
  with self.assertRaisesRegex(RuntimeError,'Board frozen arm'):
   u.check_stop_event('ARM1 EVENT COMMAND_TIMEOUT active=1 torque_off=0')
 def test_temperature_stop_is_terminal(self):
  u=UART.__new__(UART);u.motion_expected=True
  with self.assertRaisesRegex(RuntimeError,'Board stopped arm'):
   u.check_stop_event('ARM1 EVENT TEMPERATURE active=0 torque_off=1')
class SessionTests(unittest.TestCase):
 def test_xbox_disconnect_freezes_until_enter_and_resumes(self):
  import tempfile
  from unittest.mock import patch
  import right_joycon as h
  from test_trial_host import UART as MockUART
  class Source:
   disconnected=False
   def __init__(self,*args):self.n=0;self.report=(0,0,0,0)
   def read(self,*args):
    self.n+=1
    if self.n>=3 and not Source.disconnected:
     Source.disconnected=True;raise OSError('controller removed')
    return [0]*6,.5,0,False
   def close(self):pass
  class Bus(MockUART):
   history=[]
   def exchange(self,*args):return super().exchange(*args)+' COORDINATED_MANUAL_V1'
   def arm(self,c,e):
    Bus.history.append(c)
    if c=='ARM1 FREEZE':return 'ARM1 FROZEN COMMAND_TIMEOUT active=1 torque_off=0'
    if c=='ARM1 RESUME':return 'ARM1 RESUMED OK active=1 torque_off=0'
    return super().arm(c,e)
  MockUART.wrong=MockUART.failed=MockUART.lost_enable=MockUART.limit=False
  with tempfile.TemporaryDirectory() as d:
   with patch('xbox_input.XboxController',Source),patch.object(h,'UART',Bus),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
    h.main(['--source','xbox','--port','FAKE','--test-drive','--manual-control','--count','2','--csv',str(Path(d)/'out.csv')])
  self.assertIn('ARM1 FREEZE',Bus.history);self.assertIn('ARM1 RESUME',Bus.history)
  self.assertEqual(Bus.history.count('ARM1 STOP'),1)
  self.assertEqual(Bus.history[-1],'ARM1 STOP')
if __name__=='__main__':unittest.main()
