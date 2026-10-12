"""No hardware: prevent using a 135mm host with the previous firmware."""
from pathlib import Path
import sys,unittest
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
import right_joycon as host

class OldUART:
    opened=None
    def __init__(self,*args):self.closed=False;self.requests=[];OldUART.opened=self
    def exchange(self,request,*args):self.requests.append(request);return 'POSE1 READY RK7020 ARM_CONTROL_V1'
    def state(self,*args):raise AssertionError('must reject before solving or reading state')
    def close(self):self.closed=True

class IdentityTests(unittest.TestCase):
    def test_old_firmware_rejected_before_pose_or_motion(self):
        with patch.object(host,'UART',OldUART),patch.object(sys,'argv',['right_joycon.py','--port','FAKE','--count','1']):
            with self.assertRaisesRegex(RuntimeError,'requires matching SO101_135MM'):
                host.main()
        self.assertEqual(OldUART.opened.requests,[b'HELLO\n'])
        self.assertTrue(OldUART.opened.closed)
    def test_alignment_gates_remain_closed(self):
        self.assertIn('#define SO101_PHYSICAL_ALIGNMENT_VERIFIED 0',(ROOT/'firmware/servo_mapping.h').read_text(encoding='utf8'))
        self.assertIn('#define SO101_GEOMETRY_VERIFIED 0',(ROOT/'firmware/arm_alignment.h').read_text(encoding='utf8'))
    def test_uart_firmware_advertises_new_model(self):
        self.assertEqual((ROOT/'firmware/pose_uart.inc').read_text(encoding='utf8').count('SO101_135MM'),3)

if __name__=='__main__':unittest.main()
