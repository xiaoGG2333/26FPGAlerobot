"""Synchronize an independent package with a committed real-servo EEPROM baseline.

Default is validation and a dry run. This never opens a serial port or enables
model alignment. Model zeros remain candidates preserving the previous physical
count reference across the changed Homing_Offset; they are not new measured zeros.
"""
import argparse
import ast
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile

NAMES=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
SOURCE_FILES=('before.json','rk7020_right_so101.json','after_write_readback.json','summary.json')
SOURCE_PREFIX='https://github.com/JoyandAI/lerobot/blob/'
TRANSPORT='Windows DEBUG UART0 -> Zynq CAL1 -> UART1 servo bus'

def require(condition,message):
    if not condition:raise ValueError(message)

def integer(value,label,low=None,high=None):
    require(type(value) is int,label+' must be an integer')
    require(low is None or value>=low,label+' below allowed range')
    require(high is None or value<=high,label+' above allowed range')
    return value

def word(data,index):return data[index]+(data[index+1]<<8)
def signed_offset(value):
    require(0<=value<=4095,'Homing_Offset has unexpected upper bits')
    return -(value&2047) if value&2048 else value

def scalar_assignments(text):
    result={}
    for node in ast.parse(text).body:
        if isinstance(node,ast.Assign) and len(node.targets)==1 and isinstance(node.targets[0],ast.Name):
            try:result[node.targets[0].id]=ast.literal_eval(node.value)
            except (TypeError,ValueError):pass
    return result

def replace_once(text,old,new,label):
    require(text.count(old)==1,label+': expected exactly one literal match')
    return text.replace(old,new,1)

def replace_regex_once(text,pattern,replacement,label):
    matches=list(re.finditer(pattern,text,re.M|re.S))
    require(len(matches)==1,label+': expected exactly one expression match')
    m=matches[0];return text[:m.start()]+replacement+text[m.end():]

def parse_offsets(text,label):
    pattern=r'static const int offsets\[6\]=\{([^}]*)\};'
    matches=list(re.finditer(pattern,text))
    require(len(matches)==1,label+': expected exactly one offsets declaration')
    values=ast.literal_eval('['+matches[0].group(1)+']')
    require(len(values)==6 and all(type(x) is int for x in values),label+': invalid offsets')
    return values,matches[0].group(0)

def cfloat(value):
    text=f'{value:.12g}'
    if '.' not in text and 'e' not in text.lower():text+='.0'
    return text+'f'

def validate_results(result_folder):
    raw={name:(Path(result_folder)/name).read_bytes() for name in SOURCE_FILES}
    data={name:json.loads(content.decode('utf8')) for name,content in raw.items()}
    before=data['before.json'];new=data['rk7020_right_so101.json'];after=data['after_write_readback.json'];summary=data['summary.json']
    require(summary.get('status')=='CALIBRATION_PASS' and summary.get('real_calibration_complete') is True
            and summary.get('commit_confirmed') is True,'Only a completed, committed real calibration is accepted')
    require(re.fullmatch(r'COM[1-9][0-9]*',str(summary.get('port',''))) is not None,'Expected a real Windows COM port')
    for doc in (before,summary):
        require(doc.get('physical_alignment_verified') is False and doc.get('geometry_verified') is False,
                'Input must not claim physical/model alignment')
        require(not any(doc.get(key) for key in ('simulation','simulated','mock','offline','dry_run')),
                'Simulated/dry-run calibration is not accepted')
    source=before.get('source')
    require(isinstance(source,str) and re.fullmatch(re.escape(SOURCE_PREFIX)+r'[0-9a-f]{40}/src/lerobot/motors/auto_calibrate\.py',source)
            and summary.get('source')==source,
            'Pinned manufacturer source provenance missing or inconsistent')
    require(before.get('transport')==TRANSPORT,'Unexpected calibration transport')
    require(set(new)==set(NAMES) and summary.get('calibration')==new,'Calibration file and final summary disagree')
    require(isinstance(before.get('motors'),list) and len(before['motors'])==6,'Before snapshot must contain six motors')
    require(isinstance(after,list) and len(after)==6,'Final readback must contain six motors')
    old_by_id={};after_by_id={}
    for item in before['motors']:
        motor=integer(item.get('id'),'before ID',1,6)
        require(motor not in old_by_id and item.get('name')==NAMES[motor-1],'Duplicate/mismatched before identity')
        eeprom=bytes.fromhex(item.get('eeprom_hex',''));ram=bytes.fromhex(item.get('ram_hex',''))
        require(len(eeprom)==40 and len(ram)==40,'Expected complete EEPROM/RAM snapshots')
        require(word(eeprom,3)==777 and eeprom[5]==motor and eeprom[33]==0 and ram[0]==0,
                'Before snapshot model/ID/mode/torque mismatch')
        require(ram[15] in (0,1) and 90<=ram[22]<=126 and ram[23]<=55,'Invalid initial lock/health')
        offset=integer(item.get('homing_offset'),'old offset',-2047,2047)
        low=integer(item.get('range_min'),'old min',0,4095);high=integer(item.get('range_max'),'old max',0,4095)
        require(low<high and signed_offset(word(eeprom,31))==offset and word(eeprom,9)==low and word(eeprom,11)==high,
                'Before decoded EEPROM values disagree with raw bytes')
        old_by_id[motor]=item
    for item in after:
        motor=integer(item.get('id'),'after ID',1,6)
        require(motor not in after_by_id,'Duplicate final readback identity')
        after_by_id[motor]=item
    require(set(old_by_id)==set(range(1,7)) and set(after_by_id)==set(range(1,7)),'Incomplete motor IDs')
    for motor,name in enumerate(NAMES,1):
        item=new[name];observed=after_by_id[motor]
        require(integer(item.get('id'),'calibration ID',1,6)==motor and
                integer(item.get('drive_mode'),'drive mode',0,0)==0,'Calibration ID/drive mode mismatch')
        low=integer(item.get('range_min'),'new min',0,4095);high=integer(item.get('range_max'),'new max',0,4095)
        integer(item.get('homing_offset'),'new offset',-2047,2047)
        require(low<high,'New range is empty or reversed')
        require(all(type(observed.get(k)) is int and observed.get(k)==item[k] for k in ('homing_offset','range_min','range_max')),
                'Calibration/readback disagree for '+name)
        require(integer(observed.get('mode'),'final mode',0,0)==0 and integer(observed.get('torque'),'final torque',0,0)==0,
                'Final mode/torque not off')
        position=integer(observed.get('position'),'final position',-15,4110)
        require(low-15<=position<=high+15,'Final position outside accepted readback tolerance')
    return raw,data,old_by_id

@dataclass
class Plan:
    package:Path
    originals:dict
    changes:dict
    report:dict

def prepare(package,result_folder):
    package=Path(package).resolve();result_folder=Path(result_folder).resolve()
    require(package.is_dir(),'Package directory missing')
    raw,data,old_by_id=validate_results(result_folder)
    original={}
    def read(relative):
        path=(package/relative).resolve()
        require(path.is_relative_to(package),'Package source path escapes package')
        content=path.read_bytes();original[relative]=content;return content.decode('utf8')
    mapping_text=read('host/right_mapping.json');mapping=json.loads(mapping_text)
    require(mapping.get('physical_alignment_verified') is False and mapping.get('automatic_motion_enabled') is False,
            'Source package is not an unverified, inactive mapping')
    require(mapping.get('do_not_add_homing_offset_again') is True and mapping.get('counts_per_rev')==4096,
            'Unexpected encoder convention')
    require(mapping.get('joint_order')==list(NAMES[:5]) and set(mapping.get('joint_mapping',{}))==set(NAMES),
            'Unexpected source joint order')
    host=read('host/model_calibration.py');assign=scalar_assignments(host)
    limits=[(old_by_id[i]['range_min'],old_by_id[i]['range_max']) for i in range(1,7)]
    old_offsets=[old_by_id[i]['homing_offset'] for i in range(1,7)]
    require(assign.get('LIMITS')==limits and assign.get('OFFSETS')==old_offsets,'Host baseline differs from before snapshot')
    model=assign.get('MODEL');require(isinstance(model,dict) and model.get('geometry_verified_against_physical_arm') is False,
                                    'Source model geometry is not explicitly unverified')
    require(len(model.get('q_min_rad',[]))==5 and len(model.get('q_max_rad',[]))==5,'Invalid model joint bounds')
    new=data['rk7020_right_so101.json'];new_limits=[(new[n]['range_min'],new[n]['range_max']) for n in NAMES]
    offsets=[new[n]['homing_offset'] for n in NAMES];rows=[];joint_report=[]
    for i,name in enumerate(NAMES,1):
        old=mapping['joint_mapping'][name];item=new[name]
        require(old.get('id')==i and old.get('range_min')==limits[i-1][0] and old.get('range_max')==limits[i-1][1]
                and old.get('homing_offset_signed')==old_offsets[i-1],'Mapping baseline differs for '+name)
        old['homing_offset_signed']=offsets[i-1];old['range_min'],old['range_max']=new_limits[i-1]
        old['physical_zero_verified']=False;old['physical_direction_verified']=False
        if i<=5:
            require(old.get('sign') in (-1,1) and type(old.get('zero_count')) is int,'Unexpected model zero/sign')
            scale=old.get('counts_per_rad');require(isinstance(scale,(int,float)) and math.isfinite(scale) and scale>0,'Invalid scale')
            zero=(old['zero_count']+old_offsets[i-1]-offsets[i-1])%4096;sign=old['sign'];low,high=new_limits[i-1]
            bounds=sorted([sign*(low-zero)/scale,sign*(high-zero)/scale])
            qlo=max(model['q_min_rad'][i-1],bounds[0]);qhi=min(model['q_max_rad'][i-1],bounds[1])
            require(math.isfinite(qlo) and math.isfinite(qhi) and qlo<qhi,'No reachable model/count intersection for '+name)
            old.update(zero_count=zero,zero_source='previous_unverified_reference_shifted_for_new_homing_offset',
                       q_min_rad=qlo,q_max_rad=qhi,baseline_encoder_range_matched=True)
            rows.append(f'    {{{i}, {zero}, {sign}, {low}, {high}, {cfloat(qlo)}, {cfloat(qhi)}}}, /* {name} */')
            joint_report.append(dict(id=i,name=name,old_zero_candidate=json.loads(mapping_text)['joint_mapping'][name]['zero_count'],
                                     zero_candidate=zero,sign=sign,range_min=low,range_max=high,q_min=qlo,q_max=qhi))
        else:
            old['physical_endpoints_verified']=False;old['encoder_endpoints_measured']=True
            old['opening_direction_verified']=False
    mapping.update(status='EEPROM_BASELINE_MATCHED_MODEL_ALIGNMENT_UNVERIFIED',baseline_encoder_configuration_matched=True,
                   physical_alignment_verified=False,geometry_verified=False,automatic_motion_enabled=False,
                   absolute_pose_servo_drive_available=False)
    changes={}
    new_lo,new_hi=new_limits[5];old_lo,old_hi=limits[5]
    for name,low_var,high_var in (('servo_uart.inc','expected_low','expected_high'),
                               ('motor_control.c','expected_low','expected_high'),('arm_control.c','low','high')):
        relative='firmware/'+name;text=read(relative);values,declaration=parse_offsets(text,name)
        require(values==old_offsets,name+': before offsets differ')
        text=replace_once(text,declaration,'static const int offsets[6]={'+','.join(str(x) for x in offsets)+'};',name+' offsets')
        text=replace_once(text,f'{low_var}=id==6?{old_lo}:',f'{low_var}=id==6?{new_lo}:',name+' gripper min')
        text=replace_once(text,f'{high_var}=id==6?{old_hi}:',f'{high_var}=id==6?{new_hi}:',name+' gripper max')
        if name=='servo_uart.inc':
            text=replace_once(text,f'count={old_lo}..{old_hi} verified_endpoints=0',f'count={new_lo}..{new_hi} verified_endpoints=0',name+' gripper info')
        changes[relative]=text.encode('utf8')
    relative='firmware/servo_mapping.h';text=read(relative)
    require(text.count('#define SO101_PHYSICAL_ALIGNMENT_VERIFIED 0')==1,'Physical alignment flag must remain zero')
    # Check each existing C row against the unmodified JSON mapping before replacing.
    rows_found=re.findall(r'\{(\d+),\s*(\d+),\s*(-?\d+),\s*(\d+),\s*(\d+),\s*([^,{}]+),\s*([^,{}]+)\}, /\* (\w+) \*/',text)
    require(len(rows_found)==5,'Expected exactly five source C mapping rows')
    source_mapping=json.loads(mapping_text)['joint_mapping']
    for entry,name in zip(rows_found,NAMES):
        motor,zero,sign,low,high,qlo,qhi,comment=entry;old=source_mapping[name]
        require(comment==name and [int(x) for x in (motor,zero,sign,low,high)]==
                [old['id'],old['zero_count'],old['sign'],old['range_min'],old['range_max']], 'C/JSON source mapping differs')
        require(abs(float(qlo.rstrip('f'))-old['q_min_rad'])<1e-9 and abs(float(qhi.rstrip('f'))-old['q_max_rad'])<1e-9,
                'C/JSON source angular bounds differ')
    text=replace_regex_once(text,r'static const so101_joint_map so101_right_joints\[5\]=\{.*?\n\};',
                           'static const so101_joint_map so101_right_joints[5]={\n'+'\n'.join(rows)+'\n};',relative+' rows')
    changes[relative]=text.encode('utf8')
    alignment=read('firmware/arm_alignment.h')
    require(alignment.count('#define SO101_GEOMETRY_VERIFIED 0')==1,'Geometry flag must remain zero')
    relative='firmware/pose_uart.inc';text=read(relative)
    text=replace_once(text,f'lroundf({old_lo}+pose_gripper*({old_hi}-{old_lo}))',
                      f'lroundf({new_lo}+pose_gripper*({new_hi}-{new_lo}))',relative+' gripper conversion')
    changes[relative]=text.encode('utf8')
    host=replace_regex_once(host,r'^LIMITS=\[[^\n]*\]$', 'LIMITS='+repr(new_limits),'host LIMITS')
    host=replace_regex_once(host,r'^OFFSETS=\[[^\n]*\]$', 'OFFSETS='+repr(offsets),'host OFFSETS')
    ast.parse(host);changes['host/model_calibration.py']=host.encode('utf8')
    source_hashes={name:hashlib.sha256(content).hexdigest() for name,content in raw.items()}
    mapping['baseline_source']={'result_folder':str(result_folder),'source_sha256':source_hashes,
                               'zero_shift_formula':'(old_zero + old_homing_offset - new_homing_offset) % 4096'}
    changes['host/right_mapping.json']=(json.dumps(mapping,ensure_ascii=False,indent=2)+'\n').encode('utf8')
    report={'schema':'rk7020_committed_eeprom_baseline_sync_v1','result_folder':str(result_folder),
            'source':data['before.json']['source'],'source_sha256':source_hashes,
            'old_offsets':old_offsets,'new_offsets':offsets,'new_limits':new_limits,'joints':joint_report,
            'baseline_configuration_matched':True,'physical_alignment_verified':False,'geometry_verified':False,
            'absolute_pose_servo_drive_available':False,'automatic_motion_enabled':False,
            'note':'EEPROM configuration synchronization only; model zeros/directions and physical geometry remain unverified.'}
    report['previous_package_sha256']={name:hashlib.sha256(content).hexdigest() for name,content in original.items()}
    for name,content in raw.items():changes['calibration_baseline_source/'+name]=content
    changes['calibration_baseline_source/previous_mapping.json']=original['host/right_mapping.json']
    changes['baseline_sync.json']=(json.dumps(report,ensure_ascii=False,indent=2)+'\n').encode('utf8')
    for relative in changes:
        target=(package/relative).resolve();require(target.is_relative_to(package),'Target escapes package')
        if relative not in original:require(not target.exists(),'Refuse to overwrite existing baseline provenance: '+relative)
    return Plan(package,original,changes,report)

def install(plan):
    """Stage every output, verify all inputs unchanged, then install with rollback."""
    package=plan.package.resolve();stage=Path(tempfile.mkdtemp(prefix='.baseline-sync-',dir=package))
    require(stage.resolve().parent==package,'Unexpected staging directory')
    installed=[];keep_stage=False
    try:
        for relative,content in plan.changes.items():
            staged=stage/'new'/relative;staged.parent.mkdir(parents=True,exist_ok=True);staged.write_bytes(content)
            if relative in plan.originals:
                backup=stage/'old'/relative;backup.parent.mkdir(parents=True,exist_ok=True);backup.write_bytes(plan.originals[relative])
        for relative,expected in plan.originals.items():
            require((package/relative).read_bytes()==expected,'Package changed during preparation: '+relative)
        for relative in plan.changes:
            if relative not in plan.originals:require(not (package/relative).exists(),'New target appeared: '+relative)
        for relative in plan.changes:
            target=package/relative;target.parent.mkdir(parents=True,exist_ok=True)
            os.replace(stage/'new'/relative,target);installed.append(relative)
    except BaseException as failure:
        rollback_errors=[]
        for relative in reversed(installed):
            try:
                target=package/relative
                if relative in plan.originals:os.replace(stage/'old'/relative,target)
                elif target.exists():target.unlink()
            except BaseException as restore_failure:rollback_errors.append((relative,repr(restore_failure)))
        if rollback_errors:
            keep_stage=True
            raise RuntimeError(f'Install and rollback failed; preserved backups in {stage}: {rollback_errors}') from failure
        raise
    finally:
        require(stage.resolve().parent==package and stage.name.startswith('.baseline-sync-'),'Unsafe staging cleanup path')
        if not keep_stage:shutil.rmtree(stage)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package',type=Path,required=True);p.add_argument('--results',type=Path,required=True)
    p.add_argument('--apply',action='store_true',help='Install verified configuration; default validates and prints only')
    args=p.parse_args();plan=prepare(args.package,args.results)
    if args.apply:install(plan)
    print(json.dumps({'applied':args.apply,'files':sorted(plan.changes),'report':plan.report},ensure_ascii=False,indent=2))

if __name__=='__main__':main()
