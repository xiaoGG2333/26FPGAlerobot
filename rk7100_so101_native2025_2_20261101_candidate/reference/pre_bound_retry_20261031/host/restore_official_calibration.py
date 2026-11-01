"""Restore supplied R12254604 encoder parameters via CAL1. No motion targets.

Default prints the desired parameters offline. --apply backs up six motors,
writes only offset/limits/lock, verifies, then commits with torque off.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from auto_calibrate_zynq import Bridge, CalibrationError, signed, word, encode_signed

NAMES=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper']
ROOT=Path(__file__).resolve().parents[1]

def validate(data):
    if set(data)!=set(NAMES):raise ValueError('Exactly six named joints required')
    for i,n in enumerate(NAMES,1):
        j=data[n]
        if (any(type(j.get(k)) is not int for k in ('id','drive_mode','homing_offset','range_min','range_max'))
            or j['id']!=i or j['drive_mode']!=0 or not -2047<=j['homing_offset']<=2047
            or not 0<=j['range_min']<j['range_max']<=4095):raise ValueError('Invalid baseline '+n)
    return data

def restore(bus,data,folder):
    report=dict(schema='so101_official_restore_v1',started_utc=datetime.now(timezone.utc).isoformat(),
                desired=data,status='STARTED',motion_targets_sent=False,torque_enable_sent=False,
                eeprom_parameters_applied=False,physical_alignment_verified=False)
    def save():
        (folder/'restore_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf8')
    started=False;commit_attempted=False
    try:
        bus.start();started=True
        before=[]
        for i,n in enumerate(NAMES,1):
            raw=bus.read(i,0,32)+bus.read(i,32,8)
            torque=bus.read(i,40,1)[0];lock=bus.read(i,55,1)[0];feedback=bus.read(i,56,15)
            if word(raw[3:5])!=777 or raw[5]!=i or raw[33]!=0 or torque!=0 or lock not in (0,1):
                raise CalibrationError('ID%d model/ID/mode/torque/lock mismatch'%i)
            if feedback[10] or signed(feedback[2:4],15) or not 90<=feedback[6]<=126 or feedback[7]>55:
                raise CalibrationError('ID%d must be stationary and healthy'%i)
            old_offset=signed(raw[31:33],11);position=signed(feedback[:2],15)
            predicted=position+old_offset-data[n]['homing_offset']
            if not data[n]['range_min']<=predicted<=data[n]['range_max']:
                raise CalibrationError('ID%d current pose would be outside official limits after offset change; no writes made'%i)
            before.append(dict(id=i,name=n,eeprom_hex=raw.hex(),lock=lock,
                               feedback_hex=feedback.hex(),old_offset=old_offset,
                               predicted_position=predicted))
        report['before']=before;save()
        (folder/'before.json').write_text(json.dumps(before,indent=2),encoding='utf8')
        for i,n in enumerate(NAMES,1):
            j=data[n]
            if bus.read(i,40,1)!=b'\x00':raise CalibrationError('Torque unexpectedly on')
            bus.byte(i,55,0)
            # Temporary full bounds only while torque is off; no goal/speed commands.
            bus.uint(i,9,0);bus.uint(i,11,4095)
            bus.write(i,31,encode_signed(j['homing_offset'],11))
            bus.uint(i,9,j['range_min']);bus.uint(i,11,j['range_max'])
            limits=bus.read(i,9,4);offset=bus.read(i,31,2)
            if (word(limits[:2]),word(limits[2:]),signed(offset,11))!=(j['range_min'],j['range_max'],j['homing_offset']):
                raise CalibrationError('ID%d offset/limit readback mismatch'%i)
            actual=signed(bus.read(i,56,2),15)
            if abs(actual-before[i-1]['predicted_position'])>5 or not j['range_min']<=actual<=j['range_max']:
                raise CalibrationError('ID%d moved or offset convention mismatch'%i)
            if bus.read(i,40,1)!=b'\x00':raise CalibrationError('Torque unexpectedly on')
            bus.byte(i,55,before[i-1]['lock'])
        report['status']='READBACK_VERIFIED_BEFORE_COMMIT';save()
        commit_attempted=True
        report['commit_reply']=bus.finish(True);started=False
        report.update(status='OFFICIAL_ENCODER_PARAMETERS_APPLIED',eeprom_parameters_applied=True)
        save();return report
    except BaseException as error:
        report['error']=repr(error);report['status']='FAILED_COMMIT_STATE_UNCERTAIN' if commit_attempted else 'FAILED'
        if started:
            try:
                report['cleanup_reply']=bus.finish(False)
                report['status']='FAILED_COMMIT_STATE_UNCERTAIN' if commit_attempted else 'FAILED_ROLLBACK_CONFIRMED'
            except BaseException as cleanup:report['cleanup_error']=repr(cleanup)
        save();raise

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',help='Zynq DEBUG UART, not direct servo USB')
    p.add_argument('--apply',action='store_true')
    p.add_argument('--out',type=Path,default=ROOT/'results/official_restore')
    a=p.parse_args()
    path=ROOT/'official_calibration/R12254604.json'
    raw=path.read_bytes();data=validate(json.loads(raw))
    expected=json.loads((ROOT/'host/right_mapping.json').read_text())['official_encoder_source']['sha256']
    if hashlib.sha256(raw).hexdigest()!=expected:p.error('Official source changed; rebuild matching firmware first')
    if not a.apply:
        print(json.dumps(data,ensure_ascii=False,indent=2));print('OFFLINE ONLY. No serial port opened. Use --apply to restore EEPROM.');return
    if not a.port:p.error('--port required with --apply')
    if a.out.exists():p.error('Choose a new --out folder; preserve previous backups')
    a.out.mkdir(parents=True)
    bus=Bridge(a.port,a.out)
    try:
        report=restore(bus,data,a.out)
        print(report['status']);print('Torque remains OFF. Model alignment remains unverified. Backup:',a.out)
    finally:bus.close()

if __name__=='__main__':main()
