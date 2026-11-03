from pathlib import Path
import json
import csv
import math
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from right_joycon import Mapper, fk_pose
import right_joycon as h
from stick_input import StickPose
from test_trial_host import UART, Joy


class Tests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads((ROOT/'host/right_config.json').read_text())
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False

    def test_z_only_does_not_reposition_outside_workspace_x(self):
        pose=[.08,0,.2,0,0,0]
        target=Mapper(self.config,pose,[0]*6).target([0,0,.002,0,0,0],.2)
        self.assertEqual(target[:2],pose[:2])
        self.assertAlmostEqual(target[2],.202)

    def test_euler_branch_does_not_flip_yaw_by_pi(self):
        pose=fk_pose([0,.8,.6,.3,.2])
        target=Mapper(self.config,pose,[0]*6).target([0,0,.001,0,0,0],.2)
        self.assertLess(abs(h.wrap(target[5]-pose[5])),.01)

    def test_configurable_speed_and_diagonal_bound(self):
        pose=StickPose(.02).update(2500,2500,False,True,.2)[0]
        self.assertAlmostEqual(pose[2],.004)
        diagonal=StickPose(.02).update(0,4095,False,True,.2)[0]
        self.assertAlmostEqual(math.sqrt(sum(v*v for v in diagonal[:3])),.004)

    def test_lagging_elbow_stops_and_reports_axis(self):
        class LagUART(UART):
            def counts(self):return [2149,2050,2130,1985,1023,2773],0
        with tempfile.TemporaryDirectory() as folder:
            with patch.object(h,'UART',LagUART),patch.object(h,'RightJoyCon',Joy),patch('builtins.input',return_value=''):
                with self.assertRaisesRegex(RuntimeError,'ID3.*error=\\+113'):
                    h.main(['--source','joycon','--port','FAKE','--test-drive','--manual-control','--count','1','--csv',str(Path(folder)/'test.csv')])
            with (Path(folder)/'test.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(rows[-1]['status'],'FOLLOW_ERROR_ID3')
            self.assertEqual(rows[-1]['count_id3'],'2130')
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
        self.assertTrue(UART.last.closed)

    def test_unsupported_pitch_rejected_before_enabling(self):
        class PitchUART(UART):
            def state(self,*args,**kwargs):return [0]*5,[.2,0,.2,0,1.45,0]
        with patch.object(h,'UART',PitchUART),patch.object(h,'RightJoyCon',Joy):
            with self.assertRaisesRegex(RuntimeError,'pitch exceeds'):
                h.main(['--source','joycon','--port','FAKE','--test-drive','--manual-control','--count','1'])
        self.assertNotIn('ARM1 TEST_MANUAL_ENABLE',UART.last.calls)

    def test_live_shoulder_outside_model_rejected_before_enabling(self):
        class ShoulderUART(UART):
            def state(self,*args,**kwargs):
                q=[0,math.radians(-124.365234375),.7,.8,1.58]
                return q,fk_pose(q)
        with patch.object(h,'UART',ShoulderUART),patch.object(h,'RightJoyCon',Joy):
            with self.assertRaisesRegex(RuntimeError,'Starting model angle outside limits: ID2'):
                h.main(['--source','joycon','--port','FAKE','--test-drive','--manual-control','--count','1'])
        self.assertNotIn('ARM1 TEST_MANUAL_ENABLE',UART.last.calls)


if __name__=='__main__':unittest.main()
