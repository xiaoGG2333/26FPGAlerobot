"""Pure offline tests with synthetic result JSON in temporary directories.

No real package is modified and no serial module is imported. A successful
synthetic fixture exercises validation only; it is not a physical calibration.
"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parent.parent
spec=importlib.util.spec_from_file_location('sync_autocal_baseline',ROOT/'host/sync_autocal_baseline.py')
sync=importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name]=sync;spec.loader.exec_module(sync)
SOURCE=ROOT
FILES=('host/right_mapping.json','host/model_calibration.py','firmware/servo_uart.inc','firmware/motor_control.c',
       'firmware/arm_control.c','firmware/servo_mapping.h','firmware/arm_alignment.h','firmware/pose_uart.inc')

def write_json(path,value):path.write_text(json.dumps(value,indent=2),encoding='utf8')

class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='offline-baseline-test-')
        self.root=Path(self.temp.name);self.package=self.root/'package';self.results=self.root/'synthetic_results'
        self.package.mkdir();self.results.mkdir()
        for name in FILES:
            target=self.package/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(SOURCE/name,target)
        self.before=json.loads((SOURCE/'results/calibration_08/before.json').read_text(encoding='utf8'))
        self.new={};after=[]
        for item in self.before['motors']:
            motor=item['id'];offset=item['homing_offset']+10;low=item['range_min']-10;high=item['range_max']-10
            self.new[item['name']]={'id':motor,'drive_mode':0,'homing_offset':offset,'range_min':low,'range_max':high}
            after.append(dict(id=motor,homing_offset=offset,range_min=low,range_max=high,mode=0,torque=0,position=item['position']-10))
        self.after=after
        self.summary={'status':'CALIBRATION_PASS','port':'COM16','source':self.before['source'],
                      'real_calibration_complete':True,'commit_confirmed':True,
                      'physical_alignment_verified':False,'geometry_verified':False,'calibration':self.new}
        for name,value in zip(sync.SOURCE_FILES,(self.before,self.new,self.after,self.summary)):
            write_json(self.results/name,value)
    def tearDown(self):self.temp.cleanup()
    def prepare(self):return sync.prepare(self.package,self.results)
    def hashes(self):return {name:hashlib.sha256((self.package/name).read_bytes()).hexdigest() for name in FILES}
    def test_dry_plan_keeps_live_files_and_flags(self):
        original=self.hashes();plan=self.prepare();self.assertEqual(self.hashes(),original)
        mapping=json.loads(plan.changes['host/right_mapping.json'])
        self.assertFalse(mapping['physical_alignment_verified']);self.assertFalse(mapping['geometry_verified'])
        self.assertFalse(mapping['automatic_motion_enabled']);self.assertFalse(mapping['absolute_pose_servo_drive_available'])
        for i,name in enumerate(sync.NAMES[:5]):
            item=mapping['joint_mapping'][name]
            old=json.loads((self.package/'host/right_mapping.json').read_text(encoding='utf8'))['joint_mapping'][name]
            self.assertEqual(item['zero_count'],(old['zero_count']+old['homing_offset_signed']-self.new[name]['homing_offset'])%4096)
            self.assertFalse(item['physical_zero_verified']);self.assertFalse(item['physical_direction_verified'])
        self.assertIn(b'#define SO101_PHYSICAL_ALIGNMENT_VERIFIED 0',plan.changes['firmware/servo_mapping.h'])
        self.assertNotIn('firmware/arm_alignment.h',plan.changes)
    def test_apply_syncs_three_arrays_limits_gripper_and_provenance(self):
        plan=self.prepare();sync.install(plan)
        expected=[self.new[name]['homing_offset'] for name in sync.NAMES]
        for file in ('servo_uart.inc','motor_control.c','arm_control.c'):
            self.assertEqual(sync.parse_offsets((self.package/'firmware'/file).read_text(),file)[0],expected)
        host=sync.scalar_assignments((self.package/'host/model_calibration.py').read_text())
        self.assertEqual(host['OFFSETS'],expected)
        self.assertEqual(host['LIMITS'],[(self.new[n]['range_min'],self.new[n]['range_max']) for n in sync.NAMES])
        low,high=self.new['gripper']['range_min'],self.new['gripper']['range_max']
        self.assertIn(f'lroundf({low}+pose_gripper*({high}-{low}))',(self.package/'firmware/pose_uart.inc').read_text())
        for name in sync.SOURCE_FILES:
            self.assertEqual((self.package/'calibration_baseline_source'/name).read_bytes(),(self.results/name).read_bytes())
        self.assertTrue((self.package/'baseline_sync.json').exists())
        self.assertFalse(any(x.name.startswith('.baseline-sync-') for x in self.package.iterdir()))
    def test_incomplete_simulated_or_uncommitted_rejected_without_edits(self):
        original=self.hashes()
        for change in ({'commit_confirmed':False},{'status':'NOT_COMPLETED'},{'real_calibration_complete':False},
                       {'port':'mock'},{'simulation':True},{'physical_alignment_verified':True}):
            doc=copy.deepcopy(self.summary);doc.update(change);write_json(self.results/'summary.json',doc)
            with self.assertRaises(ValueError):self.prepare()
            self.assertEqual(self.hashes(),original)
        write_json(self.results/'summary.json',self.summary)
        (self.results/'after_write_readback.json').unlink()
        with self.assertRaises(FileNotFoundError):self.prepare()
    def test_duplicate_id_readback_bad_offset_and_torque_rejected(self):
        for mutate in ('duplicate','torque','mismatch','offset','range'):
            after=copy.deepcopy(self.after);new=copy.deepcopy(self.new);summary=copy.deepcopy(self.summary)
            if mutate=='duplicate':after[1]['id']=1
            elif mutate=='torque':after[1]['torque']=1
            elif mutate=='mismatch':after[1]['homing_offset']+=1
            elif mutate=='offset':new['shoulder_lift']['homing_offset']=2048
            elif mutate=='range':new['shoulder_lift']['range_min']=new['shoulder_lift']['range_max']
            summary['calibration']=new
            write_json(self.results/'after_write_readback.json',after);write_json(self.results/'rk7020_right_so101.json',new)
            write_json(self.results/'summary.json',summary)
            with self.assertRaises(ValueError):self.prepare()
    def test_late_unique_replacement_failure_has_no_partial_edits(self):
        path=self.package/'firmware/pose_uart.inc';path.write_text(path.read_text().replace('lroundf(1970+pose_gripper*(3576-1970))','lroundf(0)'))
        original=self.hashes()
        with self.assertRaises(ValueError):self.prepare()
        self.assertEqual(self.hashes(),original)
    def test_changed_source_detected_before_install(self):
        plan=self.prepare();path=self.package/'firmware/servo_mapping.h';path.write_bytes(path.read_bytes()+b'\n/* concurrent edit */\n')
        original=self.hashes()
        with self.assertRaises(ValueError):sync.install(plan)
        self.assertEqual(self.hashes(),original)
    def test_install_io_failure_restores_already_replaced_files(self):
        plan=self.prepare();original=self.hashes();replace=sync.os.replace;calls=0
        def fail_once(source,target):
            nonlocal calls;calls+=1
            if calls==3:raise OSError('Synthetic third install failure')
            return replace(source,target)
        with patch.object(sync.os,'replace',side_effect=fail_once):
            with self.assertRaises(OSError):sync.install(plan)
        self.assertEqual(self.hashes(),original)
        self.assertFalse((self.package/'baseline_sync.json').exists())

if __name__=='__main__':unittest.main(verbosity=2)
