import sys,json,tempfile,io,contextlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
r=Path(__file__).resolve().parents[1]
if not (r/'host').exists():r=Path(r'D:\VivadoProjects\RK7020_So_101')
sys.path[:0]=[str(r/'host'),str(r/'tests')]
from xbox_input import XboxController
from gripper_host import GripperClient
class Device:
 path='FAKE';name='Xbox';values={0:0,1:0,2:0,3:0,4:0,5:0}
 def capabilities(self,**kw):return {3:list(self.values)}
 def absinfo(self,k):return SimpleNamespace(value=self.values[k],min=0 if k in (2,5) else -32768,max=1023 if k in (2,5) else 32767)
 def read(self):raise BlockingIOError()
 def close(self):pass
d=Device()
with patch.dict(sys.modules,{'evdev':SimpleNamespace(InputDevice=lambda p:d)}),contextlib.redirect_stdout(io.StringIO()):
 c=XboxController('FAKE',right_axes='rxry')
 for lt,rt,expected in [(1023,0,-1),(0,1023,1),(1023,1023,0),(0,0,0),(100,0,0)]:
  d.values[2]=lt;d.values[5]=rt;pose,_,mask,_=c.read(0);assert c.gripper_direction==expected and mask==0 and pose==[0.]*6
 c.close()
print('PASS LT close, RT open, both/release HOLD, trigger deadzone, no stick/pose interference')
class U:
 def __init__(self):self.commands=[];self.rc=0
 def exchange(self,data,prefix):self.commands.append(data.decode().strip());return 'GRIP1 STATE rc=%d active=1 frozen=%d valid=1 actual=2500 goal=2500 direction=0'%(self.rc,bool(self.rc))
folder=Path.cwd()/'test_temp_gripper';folder.mkdir(exist_ok=True)
p=folder/'config.json';p.write_text(json.dumps({'verified':False}))
try:GripperClient(p)
except RuntimeError:pass
else:raise AssertionError('unverified accepted')
p.write_text(json.dumps(dict(verified=True,closed_count=2000,open_count=3500,rate_counts_s=100,servo_speed=50,torque_cap=200)))
g=GripperClient(p);u=U();g.start(u)
for direction in (-1,1,0):g.update(u,direction)
assert u.commands[-3:]==['GRIP1 INPUT CLOSE','GRIP1 INPUT OPEN','GRIP1 INPUT HOLD']
u.rc=1
try:g.update(u,0)
except RuntimeError:pass
else:raise AssertionError('timeout accepted')
assert not any('STOP' in cmd for cmd in u.commands)
print('PASS unverified calibration rejection, independent command protocol and communication-fault propagation without STOP')

import right_joycon as h
from test_trial_host import UART
class Bus(UART):
 instance=None
 def __init__(self,*a):super().__init__(*a);self.grip_commands=[];Bus.instance=self
 def exchange(self,*a):
  if a[0].startswith(b'GRIP1'):
   self.grip_commands.append(a[0].decode().strip())
   return 'GRIP1 STATE rc=0 active=1 frozen=0 valid=1 actual=2500 goal=2500 direction=0 torque_off=0'
  return super().exchange(*a)+' POSITION_ROLL_IK_20261019_V1 GRIPPER_LT_RT_20261030_V1 COORDINATED_MANUAL_V1'
 def solve(self,*a):raise AssertionError('Trigger-only input must not send a five-axis pose')
class Source:
 def __init__(self,*a):self.n=0;self.report=(0,0,0,0);self.trigger_supported=True;self.trigger_values=(0,0);self.gripper_direction=0
 def read(self,*a):
  self.n+=1
  self.trigger_values=(1,0) if self.n==3 else (1,1) if self.n==4 else (0,0)
  self.gripper_direction=-1 if self.n==3 else 0
  return [0]*6,.5,0,False
 def close(self):pass
UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
with patch.object(h,'UART',Bus),patch('xbox_input.XboxController',Source),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'),contextlib.redirect_stdout(io.StringIO()):
 h.main(['--source','xbox','--port','FAKE','--test-drive','--manual-control','--gripper-control','--gripper-config',str(p),'--count','3','--csv',str(folder/'fake.csv')])
assert Bus.instance.grip_commands[-3:]==['GRIP1 INPUT CLOSE','GRIP1 INPUT HOLD','GRIP1 INPUT HOLD']
assert Bus.instance.calls[-1]=='ARM1 STOP'
print('PASS end-to-end trigger commands while five axes idle, both/release hold and global STOP on exit')
