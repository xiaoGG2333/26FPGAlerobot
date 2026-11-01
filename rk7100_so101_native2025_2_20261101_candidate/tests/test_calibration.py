from pathlib import Path
import copy
import importlib.util
import json
import math
import shutil
import tempfile

root=Path(__file__).resolve().parents[1]
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result);return result
cal=module('cal',root/'host/model_calibration.py')
loader=module('loader',root/'apply_verified_mapping.py')
zero=[2137,2014,1955,1926,2039];sign=[-1,1,1,-1,1]
captures=[]
for degrees in (0,10):
    q=[math.radians(degrees)]*5
    captures.append(dict(schema='so101_model_reference_capture_v1',model_fingerprint=cal.FINGERPRINT,
        position_space='Present_Position_after_existing_homing_offset',limits=[list(v) for v in cal.LIMITS],homing_offsets=cal.OFFSETS,
        reference_q_rad=q,encoder_counts_median=[round(z+s*x*cal.CPR) for z,s,x in zip(zero,sign,q)]+[2428],
        encoder_counts_spread=[1]*6))
candidate=cal.fit(captures)
assert not candidate['physical_alignment_verified'] and not candidate['automatic_motion_enabled']
for i,j in enumerate(candidate['joints']):assert j['sign']==sign[i] and abs(j['zero_count']-zero[i])<1
for bad in [captures[:1],[captures[0],captures[0]],copy.deepcopy(captures)]:
    if len(bad)==2 and bad[0]!=bad[1]:bad[1]['encoder_counts_spread'][0]=10
    try:cal.fit(bad)
    except ValueError:pass
    else:raise AssertionError('Insufficient/unstable reference accepted')
with tempfile.TemporaryDirectory() as d:
    path=Path(d)/'candidate.json';path.write_text(json.dumps(candidate))
    try:loader.validate(path,root)
    except ValueError:pass
    else:raise AssertionError('Candidate must not activate motors')
    candidate.update(status='VERIFIED_MATCHING_CURRENT_HLS_MODEL',physical_alignment_verified=True,geometry_verified=True)
    for j in candidate['joints']:j.update(physical_zero_verified=True,physical_direction_verified=True)
    path.write_text(json.dumps(candidate));loader.validate(path,root)
    sandbox=Path(d)/'package';(sandbox/'host').mkdir(parents=True);(sandbox/'firmware').mkdir()
    for name in ('model_calibration.py','right_mapping.json'):shutil.copy2(root/'host'/name,sandbox/'host'/name)
    for name in ('servo_mapping.h','arm_alignment.h'):shutil.copy2(root/'firmware'/name,sandbox/'firmware'/name)
    loader.apply(path,sandbox)
    applied=json.loads((sandbox/'host/right_mapping.json').read_text(encoding='utf8'))
    assert applied['physical_alignment_verified'] and applied['geometry_verified']
    assert not applied['automatic_motion_enabled']
    assert '#define SO101_PHYSICAL_ALIGNMENT_VERIFIED 1' in (sandbox/'firmware/servo_mapping.h').read_text()
    for j in candidate['joints']:
        h=applied['joint_mapping'][j['name']]
        assert h['zero_count']==math.floor(j['zero_count']+.5) and h['sign']==j['sign']
        assert h['physical_zero_verified'] and h['physical_direction_verified']
    wrong=copy.deepcopy(candidate);wrong['position_space']='unadjusted encoder';path.write_text(json.dumps(wrong))
    try:loader.validate(path,root)
    except ValueError:pass
    else:raise AssertionError('Wrong encoder convention accepted')
    candidate['model_fingerprint']='different geometry';path.write_text(json.dumps(candidate))
    try:loader.validate(path,root)
    except ValueError:pass
    else:raise AssertionError('Different FPGA model accepted')
lines=['SERVO1 ID=1 OK raw_pos=2137 speed=0 load=0 voltage=12.2 temp=34 moving=0 current_raw=0 torque=0','SERVO1 END']
assert cal.read_position(lines,1)['raw_pos']==2137
for text in [lines[0].replace('torque=0','torque=1'),lines[0].replace('moving=0','moving=1'),lines[0].replace('12.2','8.5')]:
    try:cal.read_position([text],1)
    except RuntimeError:pass
    else:raise AssertionError('Unsafe capture accepted')
print('PASS: known-reference fitting, duplicate/unstable rejection, no fabricated verification, mismatched geometry gate, moving/energized/unhealthy capture rejection')
