import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
from right_joycon import UART

EVENT=b'ARM1 EVENT NO_MOTION_PROGRESS active=0 torque_off=1 axis=2\n'
class Serial:
    def __init__(self,lines):self.lines=iter(lines);self.reads=0
    def write(self,data):return len(data)
    def read_until(self,*a,**kw):self.reads+=1;return next(self.lines,b'')
def bus(lines):
    u=UART.__new__(UART);u.serial=Serial(lines);u.timeout=1;u.seq=0;u.motion_expected=True
    return u
class Tests(unittest.TestCase):
    def test_event_preempts_state_reply(self):
        u=bus([EVENT,b'ARM1 END\n',b'STATE1 0 0 0 0 0 0 0 0 0 0 0 0 0\n'])
        with self.assertRaisesRegex(RuntimeError,'Board stopped arm:.*axis=2'):u.state()
        self.assertEqual(u.serial.reads,1)
    def test_event_preempts_arm_reply(self):
        u=bus([EVENT,b'ARM1 END\n',b'ARM1 ALIVE OK active=1\n',b'ARM1 END\n'])
        with self.assertRaisesRegex(RuntimeError,'Board stopped arm'):u.arm('ARM1 KEEP','ALIVE')
        self.assertEqual(u.serial.reads,1)
    def test_cleanup_still_confirms_off_after_event(self):
        u=bus([EVENT,b'ARM1 END\n',b'ARM1 STOP STOPPED active=0 torque_off=1\n',b'ARM1 END\n'])
        self.assertTrue(u.arm('ARM1 STOP','STOP').startswith('ARM1 STOP '))
        self.assertFalse(u.motion_expected)
    def test_ready_starts_guard_and_active_event_is_allowed(self):
        u=bus([b'ARM1 READY TEST\n',b'ARM1 END\n']);u.motion_expected=False
        u.arm('ARM1 TEST_MANUAL_ENABLE','READY')
        self.assertTrue(u.motion_expected)
        u.check_stop_event('ARM1 EVENT OUT active=1 torque_off=0')
if __name__=='__main__':unittest.main()
