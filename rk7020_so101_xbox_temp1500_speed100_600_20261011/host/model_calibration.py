"""Read-only reference-pose capture and candidate encoder mapping. No motor writes."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import statistics
import time
import serial

NAMES=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll']
MODEL={'tx_m':[.0612,.02943,.02798,.13504,.0593],
       'tz_m':[.0598,.05504,.1127,.00519,.00996],
       'axes':['z','y','y','y','x'],
       'q_min_rad':[-1.2778059963087391, -2.021786678433275, -1.5447186534008406, -1.6904468282499765, -2.9437091319525455],
       'q_max_rad':[1.4526798061277022, 1.581534192310096, 1.7947575218262002, 1.8162332528565992, 2.8486023231036355],
       'geometry_verified_against_physical_arm':False}
FINGERPRINT=hashlib.sha256(json.dumps(MODEL,sort_keys=True).encode()).hexdigest()
CPR=4096/(2*math.pi)
LIMITS=[(1164, 2964), (828, 3197), (876, 3073), (874, 3180), (170, 3966), (1975, 3567)]
OFFSETS=[-1429, -1868, -551, 1828, -773, 1727]

def request(uart,command,marker,timeout=5):
    uart.write((command+'\n').encode('ascii'))
    deadline=time.monotonic()+timeout
    buffer=bytearray();lines=[]
    while time.monotonic()<deadline:
        ch=uart.read(1)
        if not ch:continue
        if ch==b'\n':
            line=buffer.decode('ascii',errors='replace').strip();buffer.clear()
            if line:lines.append(line)
            if line==marker or (marker=='POSE1 READY' and line.startswith('POSE1 READY ')):
                return lines
        elif len(buffer)<4096:buffer.extend(ch)
        else:raise RuntimeError('Oversized serial reply')
    raise TimeoutError('No complete response to '+command)

def read_position(lines,motor_id):
    pattern=rf'^SERVO1 ID={motor_id} OK raw_pos=(-?\d+) speed=(-?\d+) load=(-?\d+) voltage=([\d.]+) temp=(\d+) moving=(\d+) current_raw=(-?\d+) torque=(\d+)$'
    matches=[re.fullmatch(pattern,line) for line in lines]
    matches=[m for m in matches if m]
    if len(matches)!=1:raise RuntimeError('Missing/unexpected feedback from ID'+str(motor_id))
    pos,speed,load,voltage,temp,moving,current,torque=matches[0].groups()
    data=dict(id=motor_id,raw_pos=int(pos),speed=int(speed),load=int(load),voltage=float(voltage),
              temp=int(temp),moving=int(moving),current_raw=int(current),torque=int(torque))
    low,high=LIMITS[motor_id-1]
    if not low<=data['raw_pos']<=high:raise RuntimeError('Encoder outside configured range')
    if data['torque'] or data['moving']:raise RuntimeError('All servos must be stationary and torque off')
    if not 9<=data['voltage']<=12.6 or data['temp']>55:raise RuntimeError('Unhealthy voltage or temperature')
    return data

def collect(port,samples=5,interval=.1):
    record={'schema':'so101_encoder_snapshot_v1','captured_utc':datetime.now(timezone.utc).isoformat(),
            'port':port,'model':MODEL,'model_fingerprint':FINGERPRINT,
            'joint_order':NAMES,'position_space':'Present_Position_after_existing_homing_offset',
            'limits':LIMITS,'homing_offsets':OFFSETS,'physical_alignment_verified':False,
            'motion_commands_sent':False,'samples':[]}
    with serial.Serial(port,115200,timeout=.1,write_timeout=2) as uart:
        hello=request(uart,'HELLO','POSE1 READY')
        if not any('SINGLE_MOTOR_JOG_POSE_PREVIEW' in line or 'ARM_CONTROL_V1' in line for line in hello):
            raise RuntimeError('Expected new RK7020 single-motor jog firmware')
        record['hello']=hello
        state=request(uart,'MOTOR1 STATE','MOTOR1 END')
        if not any(line.startswith('MOTOR1 STATE active=0 ') for line in state):
            raise RuntimeError('Another motion is active')
        calibration=request(uart,'SERVO1 CALCHECK','SERVO1 END')
        if not any(line.startswith('SERVO1 CALCHECK PASS ') for line in calibration):
            raise RuntimeError('Servo EEPROM configuration differs from mapping')
        record['calcheck']=calibration
        for sample in range(samples):
            readings=[read_position(request(uart,f'SERVO1 READ {i}','SERVO1 END'),i) for i in range(1,7)]
            record['samples'].append(readings)
            if sample+1<samples:time.sleep(interval)
    spreads=[max(s[i]['raw_pos'] for s in record['samples'])-min(s[i]['raw_pos'] for s in record['samples']) for i in range(6)]
    if max(spreads)>5:raise RuntimeError('Pose moved during capture; hold it steady and capture again')
    record['encoder_counts_median']=[statistics.median(s[i]['raw_pos'] for s in record['samples']) for i in range(6)]
    record['encoder_counts_spread']=spreads
    return record

def fit(captures):
    if len(captures)<2:raise ValueError('At least two distinct known reference poses required')
    for c in captures:
        if c.get('schema')!='so101_model_reference_capture_v1' or c.get('model_fingerprint')!=FINGERPRINT:
            raise ValueError('Only known reference captures for this same model are accepted')
        if c.get('position_space')!='Present_Position_after_existing_homing_offset':
            raise ValueError('Unexpected encoder position convention')
        if c.get('limits')!=[list(x) for x in LIMITS] or c.get('homing_offsets')!=OFFSETS:
            raise ValueError('Servo configuration changed between captures')
        angles=c.get('reference_q_rad',[]);counts=c.get('encoder_counts_median',[])
        spreads=c.get('encoder_counts_spread',[])
        if len(angles)!=5 or len(counts)!=6 or len(spreads)!=6:
            raise ValueError('Incomplete capture')
        if not all(math.isfinite(float(v)) for v in angles+counts+spreads) or max(spreads)>5 or min(spreads)<0:
            raise ValueError('Non-finite or unstable capture')
        if any(not LIMITS[i][0]<=counts[i]<=LIMITS[i][1] for i in range(6)):
            raise ValueError('Captured encoder outside limits')
    joints=[]
    for i,name in enumerate(NAMES):
        x=[float(c['reference_q_rad'][i]) for c in captures]
        y=[float(c['encoder_counts_median'][i]) for c in captures]
        if max(x)-min(x)<math.radians(5):raise ValueError(name+': known angle separation must be at least 5 degrees')
        xm,ym=statistics.mean(x),statistics.mean(y)
        slope=sum((a-xm)*(b-ym) for a,b in zip(x,y))/sum((a-xm)**2 for a in x)
        if abs(abs(slope)/CPR-1)>.1:raise ValueError(name+': count/angle slope differs by more than 10%; check pose angles')
        sign=1 if slope>0 else -1
        zero=statistics.median(b-sign*a*CPR for a,b in zip(x,y))
        errors=[b-(zero+sign*a*CPR) for a,b in zip(x,y)]
        if max(abs(e) for e in errors)>8:raise ValueError(name+': residual over 8 counts')
        if not LIMITS[i][0]<=zero<=LIMITS[i][1]:raise ValueError(name+': zero outside hardware range')
        joints.append(dict(name=name,id=i+1,zero_count=zero,sign=sign,counts_per_rad=CPR,
                           maximum_residual_counts=max(abs(e) for e in errors),
                           measured_slope_counts_per_rad=slope,physical_zero_verified=False,
                           physical_direction_verified=False))
    return {'schema':'so101_reference_mapping_candidate_v1','status':'CANDIDATE_REQUIRES_PHYSICAL_AND_MODEL_REVIEW',
            'model':MODEL,'model_fingerprint':FINGERPRINT,'joint_order':NAMES,'joints':joints,
            'reference_capture_count':len(captures),'physical_alignment_verified':False,
            'automatic_motion_enabled':False,'position_space':'Present_Position_after_existing_homing_offset',
            'limits':[list(x) for x in LIMITS],'homing_offsets':OFFSETS}

def main():
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='action',required=True)
    for name in ('snapshot','capture'):
        s=sub.add_parser(name);s.add_argument('--port',required=True);s.add_argument('--out',type=Path,required=True)
        s.add_argument('--samples',type=int,default=5)
        if name=='capture':
            s.add_argument('--label',required=True)
            s.add_argument('--q-deg',nargs=5,type=float,required=True,
                           help='Operator-known physical model angles; not a desired motion target')
    s=sub.add_parser('fit');s.add_argument('captures',nargs='+',type=Path);s.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():p.error('Refuse to overwrite an existing capture/output file')
    if a.action=='fit':
        data=fit([json.loads(c.read_text(encoding='utf8')) for c in a.captures])
    else:
        if not 3<=a.samples<=30:p.error('samples must be 3..30')
        if a.action=='capture':
            if not all(math.isfinite(v) for v in a.q_deg):p.error('angles must be finite')
            q=[math.radians(v) for v in a.q_deg]
            if any(not MODEL['q_min_rad'][i]<=q[i]<=MODEL['q_max_rad'][i] for i in range(5)):
                p.error('Reference angle exceeds this model range')
        data=collect(a.port,a.samples)
        if a.action=='capture':
            data.update(schema='so101_model_reference_capture_v1',label=a.label,
                        reference_q_rad=q,reference_q_deg=a.q_deg,
                        reference_angles_source='operator_supplied_not_independently_measured')
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf8')
    print('Saved',a.out,'; physical alignment remains unverified; no motor/EEPROM writes.')

if __name__=='__main__':
    try:main()
    except (OSError,RuntimeError,ValueError,TimeoutError,serial.SerialException) as error:
        raise SystemExit(str(error))

