"""No serial device opened; actual UART decoder exercised with fake bytes."""
from pathlib import Path
import sys,unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from right_joycon import UART,parse_result
def frame(n,code=1):return 'RESULT1 1 %d %d .001 0 0 -.2 .3 .1 0 .5'%(code,n)
class Serial:
 def __init__(self,lines):self.lines=iter((s+'\n').encode() for s in lines)
 def write(self,b):return len(b)
 def read_until(self,*a,**kw):return next(self.lines)
class Tests(unittest.TestCase):
 def test_bounds(self):
  for code in [0,1,2,3,4]:
   for n in [0,200,242,360]:self.assertEqual(parse_result(frame(n,code),1,360)['iterations'],n)
  for n in [-1,361,600]:
   with self.assertRaises(ValueError):parse_result(frame(n),1,360)
  with self.assertRaises(ValueError):parse_result(frame(242),1)
  with self.assertRaises(ValueError):parse_result(frame(242).replace('.001','nan'),1,360)
  with self.assertRaises(ValueError):parse_result(frame(242),2,360)
 def test_negotiated_exchange(self):
  for capability,limit in [(' BOUND_RETRY_IK_20261031_V1',360),('',200)]:
   u=UART.__new__(UART);u.serial=Serial(['POSE1 READY'+capability,frame(242)]);u.timeout=.2;u.seq=0
   u.exchange(b'HELLO\n','POSE1 READY')
   self.assertEqual(u.result_iteration_limit,limit)
   if limit==360:self.assertEqual(u.solve(0,[.2,0,.2,0,0,0],.5)['status'],'MAX_ITER')
   else:
    with self.assertRaises(ValueError):u.solve(0,[.2,0,.2,0,0,0],.5)
unittest.main()
