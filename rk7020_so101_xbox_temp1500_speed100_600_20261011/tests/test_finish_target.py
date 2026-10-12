import csv, json, sys, tempfile, unittest
from pathlib import Path
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from finish_target import CompletionTracker
import right_joycon as h
from test_trial_host import UART, Joy

class Tests(unittest.TestCase):
    def test_arrival_needs_all_axes_three_samples_and_unchanged_goal(self):
        t=CompletionTracker();goal=[2000]*5
        self.assertEqual(t.observe([2000,2000,1980,2000,2000],goal),(False,20))
        self.assertFalse(t.observe([1995]*5,goal)[0])
        self.assertFalse(t.observe([1995]*5,goal)[0])
        self.assertTrue(t.observe([1995]*5,goal)[0])
        self.assertFalse(t.observe([2000]*5,[2001]*5)[0])
        self.assertFalse(t.observe([1900]*5,[2001]*5)[0])
        self.assertEqual(t.samples,0)
    def test_bad_feedback_rejected(self):
        for actual in ([2000]*4,[4096]*5,[2000.0]*5):
            with self.assertRaises(ValueError):CompletionTracker().observe(actual,[2000]*5)
    def test_release_preserves_goal_until_feedback_arrives(self):
        class Bus(UART):
            def __init__(self,*a):
                super().__init__(*a);self.actual=[2149,2050,2017,1985,1023];self.goal=list(self.actual);self.n=0
            def exchange(self,*a):return super().exchange(*a)+' COORDINATED_MANUAL_V1'
            def solve(self,*a):
                self.goal[1]=2090
                return super().solve(*a)
            def counts(self):return self.goal+[2773],0
            def arm(self,c,e):
                if c=='ARM1 STATE':
                    self.calls.append(c)
                    self.n+=1
                    self.actual[1]=2050+([0,8,16,40,40,40][min(self.n-1,5)])
                    return 'ARM1 STATE active=1 mode=3 aligned=0 geometry=0 result=OK torque_off=0 counts='+','.join(map(str,self.actual))+' goal='+','.join(map(str,self.goal))
                return super().arm(c,e)
        class Input(Joy):
            n=0
            def read(self,t):
                self.n+=1
                return ([.004,0,0,0,0,0] if self.n>=3 else [0]*6),.5,(1 if self.n==3 else 0),False
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'out.csv'
            with patch.object(h,'UART',Bus),patch('stick_input.StickJoyCon',Input),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','stick','--port','FAKE','--test-drive','--manual-control','--count','6','--csv',str(out)])
            with out.open() as f:rows=list(csv.DictReader(f))
        self.assertEqual([r['status'] for r in rows],['CONVERGED']+['STICK_SETTLING']*4+['STICK_HOLD'])
        self.assertTrue(all(r['count_id2']=='2090' for r in rows))
        self.assertNotIn('ARM1 PAUSE',UART.last.calls)
        self.assertEqual(UART.last.seq,1)
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
        self.assertTrue(all(None not in r for r in rows))
    def test_fault_during_release_stops_and_closes(self):
        class Fault(UART):
            def arm(self,c,e):
                if c=='ARM1 KEEP':return 'ARM1 EVENT NO_MOTION_PROGRESS active=0 torque_off=1'
                return super().arm(c,e)
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',Fault),patch('stick_input.StickJoyCon',Joy),patch('builtins.input',return_value=''):
                with self.assertRaisesRegex(RuntimeError,'Arm session stopped'):
                    h.main(['--source','stick','--port','FAKE','--test-drive','--manual-control','--count','1','--csv',str(Path(d)/'out.csv')])
        self.assertEqual(UART.last.calls[-1],'ARM1 STOP')
        self.assertTrue(UART.last.closed)

    def test_new_input_after_arrival_uses_measured_origin_once(self):
        mapping=json.loads((ROOT/'host/right_mapping.json').read_text())
        joints=[mapping['joint_mapping'][name] for name in mapping['joint_order']]
        base=[2149,2050,2017,1985,1023]
        arrived=[2149,2090,2017,1985,1023]
        def q_from(c):return [j['sign']*(v-j['zero_count'])/j['counts_per_rad'] for j,v in zip(joints,c)]
        class Bus(UART):
            def __init__(self,*a):super().__init__(*a);self.targets=[];self.actual=list(base);self.goal=list(base)
            def exchange(self,*a):return super().exchange(*a)+' COORDINATED_MANUAL_V1'
            def state(self,*a,**kw):q=q_from(self.actual);return q,h.fk_pose(q)
            def solve(self,core,target,gripper):
                self.targets.append(list(target));self.goal=list(arrived)
                return super().solve(core,target,gripper)
            def counts(self):return self.goal+[2773],0
            def arm(self,c,e):
                if c=='ARM1 STATE':
                    self.calls.append(c)
                    if self.targets:self.actual=list(arrived)
                    return 'ARM1 STATE active=1 mode=3 aligned=0 geometry=0 result=OK torque_off=0 counts='+','.join(map(str,self.actual))+' goal='+','.join(map(str,self.goal))
                return super().arm(c,e)
        class Input(Joy):
            n=0
            def read(self,t):
                self.n+=1
                x=0 if self.n<3 else .004 if self.n<7 else .008
                return [x,0,0,0,0,0],.5,1 if self.n in (3,7) else 0,False
        UART.wrong=UART.failed=UART.lost_enable=UART.limit=False
        with tempfile.TemporaryDirectory() as d:
            with patch.object(h,'UART',Bus),patch('stick_input.StickJoyCon',Input),patch('builtins.input',return_value=''),patch.object(h.time,'sleep'):
                h.main(['--source','stick','--port','FAKE','--test-drive','--manual-control','--count','5','--csv',str(Path(d)/'out.csv')])
        self.assertEqual(len(UART.last.targets),2)
        self.assertAlmostEqual(UART.last.targets[0][0],h.fk_pose(q_from(base))[0]+.004)
        self.assertAlmostEqual(UART.last.targets[1][0],h.fk_pose(q_from(arrived))[0]+.004)
        self.assertNotIn('ARM1 PAUSE',UART.last.calls)

if __name__=='__main__':unittest.main()
