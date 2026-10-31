import sys,unittest,io
from pathlib import Path
from contextlib import redirect_stdout
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from chip_telemetry import parse_thermal,consume_thermal,thermal_values
class ThermalTests(unittest.TestCase):
 def test_normal(self):
  d=parse_thermal('THERMAL1 42 chip_c=42.5 fan_pct=50 rpm=2700 state=NORMAL sensor_ok=1');self.assertEqual(d['fan_pwm_pct'],50);self.assertEqual(d['seq'],42)
 def test_fault(self):
  d=parse_thermal('THERMAL1 0 chip_c=nan fan_pct=100 rpm=0 state=SENSOR_FAULT sensor_ok=0');self.assertFalse(d['chip_sensor_ok'])
 def test_reject(self):
  for line in ['THERMAL1 0 chip_c=nan fan_pct=50 rpm=0 state=NORMAL sensor_ok=1','THERMAL1 0 chip_c=75 fan_pct=50 rpm=0 state=HOT_HOLD sensor_ok=1','THERMAL1 0 chip_c=30 fan_pct=25 rpm=0 state=NORMAL sensor_ok=1']:
   with self.assertRaises(ValueError):parse_thermal(line)
 def test_sideband(self):
  class U:pass
  u=U()
  with redirect_stdout(io.StringIO()):self.assertTrue(consume_thermal(u,'THERMAL1 7 chip_c=67 fan_pct=100 rpm=4000 state=FULL sensor_ok=1'))
  self.assertEqual(thermal_values(u),[67.,100,4000,'FULL',1]);self.assertFalse(consume_thermal(u,'RESULT1 7 0'))
 def test_absent(self):self.assertEqual(thermal_values(None),['']*5)

 def uart(self,lines):
  from right_joycon import UART
  class Serial:
   def __init__(self):self.lines=[(x+'\n').encode() for x in lines]
   def write(self,x):return len(x)
   def read_until(self,*args,**kwargs):return self.lines.pop(0) if self.lines else b''
  u=UART.__new__(UART);u.serial=Serial();u.timeout=.2;u.seq=0;u.motion_expected=True;return u
 def test_exchange_interleaving(self):
  u=self.uart(['THERMAL1 0 chip_c=42.5 fan_pct=50 rpm=2500 state=NORMAL sensor_ok=1','STATE1 0 '+'0 '*11+'0'])
  with redirect_stdout(io.StringIO()):q,pose=u.state()
  self.assertEqual(len(q),5);self.assertEqual(u.thermal['fan_rpm'],2500)
 def test_arm_interleaving(self):
  u=self.uart(['THERMAL1 0 chip_c=68 fan_pct=100 rpm=4000 state=FULL sensor_ok=1','ARM1 ALIVE OK active=1 torque_off=0','ARM1 END'])
  with redirect_stdout(io.StringIO()):line=u.arm('ARM1 KEEP','ALIVE')
  self.assertIn('ALIVE',line);self.assertEqual(u.thermal['fan_pwm_pct'],100)
 def test_hot_is_freeze_not_stop(self):
  u=self.uart(['THERMAL1 0 chip_c=76 fan_pct=100 rpm=4000 state=HOT_HOLD sensor_ok=1','ARM1 EVENT CHIP_OVERHEAT_OR_SENSOR_FAULT active=1 torque_off=0'])
  with redirect_stdout(io.StringIO()):
   with self.assertRaisesRegex(RuntimeError,'Board frozen arm:'):u.arm('ARM1 KEEP','ALIVE')

if __name__=='__main__':unittest.main()
