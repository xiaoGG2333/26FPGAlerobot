from pathlib import Path
import json
import sys
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
import diagnose_teleop as d


class ReadOnlyUART:
    last=None
    def __init__(self,*args):
        self.commands=[];self.closed=False;ReadOnlyUART.last=self
    def exchange(self,request,prefix):
        self.commands.append(request.decode().strip())
        if request==b'HELLO\n':return 'POSE1 READY SO101_135MM'
        motor=int(request.split()[-1])
        assert request==('SERVO1 READ %d\n'%motor).encode()
        zero=[2121,2156,1893,1986,2037,2428][motor-1]
        return 'SERVO1 ID=%d OK raw_pos=%d speed=0 load=0 voltage=12.2 temp=34 moving=0 current_raw=0 torque=0'%(motor,zero)
    def arm(self,command,expected):
        self.commands.append(command)
        assert command=='ARM1 STATE'
        return 'ARM1 STATE active=0 mode=0'
    def close(self):self.closed=True


class Tests(unittest.TestCase):
    def setUp(self):
        self.mapping=json.loads((ROOT/'host/right_mapping.json').read_text())
    def test_stationary_diagnosis_uses_only_reads(self):
        with patch.object(d,'UART',ReadOnlyUART):
            report=d.live_report('FAKE',1,False,self.mapping)
        self.assertEqual(report['status'],'PASS')
        self.assertTrue(report['healthy_stationary'])
        self.assertEqual(report['q_deg'],[0.]*5)
        self.assertEqual(ReadOnlyUART.last.commands,['HELLO','ARM1 STATE']+['SERVO1 READ %d'%i for i in range(1,7)])
        self.assertTrue(ReadOnlyUART.last.closed)
    def test_recorded_r_input_reproduces_opposite_model_motion(self):
        old_mapping=json.loads((ROOT/'official_calibration/previous_mapping.json').read_text())
        report=d.analyze_csv(ROOT/'results/r_only_check.csv',old_mapping)
        self.assertEqual(report['max_follow_error_counts']['elbow_flex'],113)
        run=report['r_only_runs'][0]
        self.assertAlmostEqual(run['requested_dz_mm'],18)
        self.assertLess(run['encoder_model_dz_mm'],-12)

    def test_live_shoulder_pose_is_outside_model(self):
        readiness=d.pose_readiness([2127,839,2210,2575,1023],self.mapping)
        self.assertFalse(readiness['pose_in_model_limits'])
        issue=readiness['issues'][0]
        self.assertEqual(issue['id'],2)
        self.assertEqual(issue['reason'],'MODEL_ANGLE_LIMIT')


if __name__=='__main__':unittest.main()

