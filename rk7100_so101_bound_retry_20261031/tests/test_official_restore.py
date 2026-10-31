from pathlib import Path
import sys,json,tempfile,unittest
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'host'))
from restore_official_calibration import restore,validate
from auto_calibrate_zynq import signed,encode_signed

class Fake:
    def __init__(self,fail=False):
        self.memory=[];self.writes=[];self.fail=fail
        old=[-1442,-1877,-527,1776,-745,1714]
        for i in range(6):
            m=bytearray(80);m[3:5]=(777).to_bytes(2,'little');m[5]=i+1
            m[9:11]=(100).to_bytes(2,'little');m[11:13]=(4000).to_bytes(2,'little')
            m[31:33]=encode_signed(old[i],11);m[55]=1;m[62]=121;m[63]=31
            self.memory.append(m)
        self.saved=[bytes(m) for m in self.memory];self.old=old
    def start(self):pass
    def read(self,i,a,n):
        m=self.memory[i-1]
        pos=2200+self.old[i-1]-signed(m[31:33],11)
        m[56:58]=pos.to_bytes(2,'little')
        return bytes(m[a:a+n])
    def write(self,i,a,b):
        self.writes.append((i,a,bytes(b)))
        if a not in (9,11,31,55):raise AssertionError('Unexpected motor setting or motion write')
        if self.fail and i==3 and a==31:raise RuntimeError('Injected lost write acknowledgement')
        self.memory[i-1][a:a+len(b)]=b
    def byte(self,i,a,v):self.write(i,a,bytes([v]))
    def uint(self,i,a,v):self.write(i,a,v.to_bytes(2,'little'))
    def finish(self,commit):
        if not commit:self.memory=[bytearray(m) for m in self.saved]
        return ['CAL1 COMMIT off_confirmed=1 rollback_success=1' if commit else 'CAL1 STOP off_confirmed=1 rollback_success=1']

class Tests(unittest.TestCase):
    def setUp(self):self.data=validate(json.loads((ROOT/'official_calibration/R12254604.json').read_text()))
    def test_apply_exact_and_no_motion(self):
        b=Fake()
        with tempfile.TemporaryDirectory() as d:r=restore(b,self.data,Path(d))
        self.assertTrue(r['eeprom_parameters_applied'])
        for i,n in enumerate(self.data):
            j=self.data[n];m=b.memory[i]
            self.assertEqual(signed(m[31:33],11),j['homing_offset'])
            self.assertEqual(int.from_bytes(m[9:11],'little'),j['range_min'])
            self.assertEqual(int.from_bytes(m[11:13],'little'),j['range_max'])
            self.assertEqual(m[40],0);self.assertEqual(m[55],1)
    def test_failure_rolls_back_all_motors(self):
        b=Fake(True)
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError,'Injected'):restore(b,self.data,Path(d))
            r=json.loads((Path(d)/'restore_report.json').read_text())
            self.assertEqual(r['status'],'FAILED_ROLLBACK_CONFIRMED')
        self.assertEqual([bytes(m) for m in b.memory],b.saved)
    def test_torque_on_prevents_parameter_writes(self):
        b=Fake();b.memory[2][40]=1
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError,'torque'):restore(b,self.data,Path(d))
        self.assertEqual(b.writes,[])
    def test_rebase_preserves_model_angles(self):
        old=json.loads((ROOT/'official_calibration/previous_mapping.json').read_text())
        new=json.loads((ROOT/'official_calibration/previous_project_mapping.json').read_text())
        for n in old['joint_order']:
            a=old['joint_mapping'][n];b=new['joint_mapping'][n]
            for c in (a['zero_count']-100,a['zero_count'],a['zero_count']+100):
                rebased=c+a['homing_offset_signed']-b['homing_offset_signed']
                self.assertAlmostEqual(a['sign']*(c-a['zero_count'])/a['counts_per_rad'],
                                       b['sign']*(rebased-b['zero_count'])/b['counts_per_rad'])

if __name__=='__main__':unittest.main()
