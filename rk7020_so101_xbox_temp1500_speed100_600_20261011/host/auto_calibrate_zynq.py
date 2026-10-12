"""Bounded manufacturer-style SO101 calibration through RK7020 CAL1 UART bridge.

No native LeRobot connect() or automatic all-axis old-goal enable is used.
Endpoint/offset mathematics and action order come from the pinned vendor fork.
"""
import argparse
from datetime import datetime, timezone, timedelta
import json
import math
from pathlib import Path
import time
import serial
from vendor_calibration_math import (ORDER, MOTOR_IDS, DIRECTIONS, PRE_ACTIONS,
                                     POST_ACTIONS, RECOVERY_ORDER, SOURCE_URL,
                                     compute_calibration)

class CalibrationError(RuntimeError):pass

def word(data):return int.from_bytes(data,'little')
def signed(data,bit):
    value=word(data);return -(value&((1<<bit)-1)) if value&(1<<bit) else value
def encode_signed(value,bit):return ((abs(value)|(1<<bit)) if value<0 else value).to_bytes(2,'little')
def wrapped_delta(current,previous):return (current-previous+2048)%4096-2048

class Bridge:
    def __init__(self,port,folder,serial_factory=serial.Serial):
        self.serial=serial_factory(port,115200,timeout=.05,write_timeout=2)
        self.seq=0;self.active=False;self.trace_supported=False
        self.stop_path=folder/'STOP'
        self.log=(folder/'transactions.jsonl').open('w',encoding='utf8')
    def exchange(self,command,hello=False):
        diagnostic_after_stop=(not self.active and command.startswith('CAL1 TRACE '))
        if command!='CAL1 STOP' and not diagnostic_after_stop and self.stop_path.exists():
            raise CalibrationError('Operator STOP file detected')
        self.serial.write((command+'\n').encode('ascii'))
        # A failed bus write can invoke the firmware's bounded six-axis rollback.
        # Allow that cleanup response to arrive; the board watchdog remains local.
        deadline=time.monotonic()+15.;lines=[];buffer=bytearray()
        while time.monotonic()<deadline:
            chunk=self.serial.read(1)
            if not chunk:continue
            if chunk!=b'\n':
                buffer.extend(chunk)
                if len(buffer)>1024:raise CalibrationError('Oversized UART frame')
                continue
            line=buffer.decode('ascii',errors='replace').strip();buffer.clear()
            if not line:continue
            lines.append(line)
            if (hello and line.startswith('POSE1 READY ')) or (not hello and line=='CAL1 END'):
                self.log.write(json.dumps({'time':time.time(),'command':command,'reply':lines})+'\n');self.log.flush()
                if any(x.startswith('CAL1 ERROR ') or x.startswith('CAL1 EVENT ') for x in lines):
                    raise CalibrationError(' | '.join(lines))
                return lines
        self.log.write(json.dumps({'time':time.time(),'command':command,'reply':lines,'timeout':True})+'\n');self.log.flush()
        raise CalibrationError('Unconfirmed UART transaction; no retry: '+command)
    def start(self):
        hello=self.exchange('HELLO',True)
        if not any('AUTOCAL_BRIDGE_V1' in x and 'ATOMIC_HOLD' in x for x in hello):
            raise CalibrationError('AUTOCAL_BRIDGE_V1 ATOMIC_HOLD firmware required')
        self.trace_supported=any('BUS_TRACE_V1' in x for x in hello)
        lines=self.exchange('CAL1 BEGIN')
        if not any(x.startswith('CAL1 READY active=1 ') for x in lines):raise CalibrationError('Calibration session not ready')
        self.active=True;self.seq=0
    def read(self,motor,addr,size):
        self.seq+=1;lines=self.exchange(f'CAL1 READ {self.seq} {motor} {addr} {size}')
        values=[x.split()[3] for x in lines if x.startswith(f'CAL1 DATA {self.seq} ')]
        if len(values)!=1:raise CalibrationError('Missing register read response')
        result=bytes.fromhex(values[0])
        if len(result)!=size:raise CalibrationError('Wrong register read size')
        return result
    def write(self,motor,addr,data):
        self.seq+=1;lines=self.exchange(f'CAL1 WRITE {self.seq} {motor} {addr} {bytes(data).hex().upper()}')
        if f'CAL1 OK {self.seq}' not in lines:raise CalibrationError('Register write not confirmed')
    def byte(self,motor,addr,value):self.write(motor,addr,bytes([value]))
    def uint(self,motor,addr,value):self.write(motor,addr,int(value).to_bytes(2,'little'))
    def hold(self,motor):
        self.seq+=1;lines=self.exchange(f'CAL1 HOLD {self.seq} {motor}')
        if f'CAL1 OK {self.seq}' not in lines:raise CalibrationError('Atomic current-position hold not confirmed')
    def keep(self):
        if 'CAL1 KEEP active=1' not in self.exchange('CAL1 KEEP'):raise CalibrationError('Lost calibration session')
    def wait(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            time.sleep(min(.2,max(0,end-time.monotonic())));self.keep()
    def finish(self,commit):
        action='COMMIT' if commit else 'STOP';lines=self.exchange('CAL1 '+action)
        if not any(x.startswith('CAL1 '+action+' ') and 'off_confirmed=1' in x and 'rollback_success=1' in x for x in lines):
            raise CalibrationError('STOP/configuration cleanup not confirmed: '+' | '.join(lines))
        self.active=False;return lines
    def capture_trace(self,folder):
        # Retrieval is memory-only and is allowed only after confirmed cleanup.
        # Never let a diagnostic failure hide the original calibration failure.
        if not self.trace_supported:return {'status':'UNSUPPORTED'}
        if self.active:return {'status':'SKIPPED_ACTIVE'}
        lines=self.exchange('CAL1 TRACE COUNT')
        counts=[int(x.split()[3]) for x in lines if x.startswith('CAL1 TRACE COUNT ')]
        if len(counts)!=1 or not 0<=counts[0]<=32:raise CalibrationError('Invalid trace count')
        rows=[]
        for index in range(counts[0]):
            reply=self.exchange(f'CAL1 TRACE {index}')
            matches=[x for x in reply if x.startswith(f'CAL1 TRACE {index} ')]
            if len(matches)!=1:raise CalibrationError('Missing trace record')
            fields=dict(x.split('=',1) for x in matches[0].split()[3:])
            for key in ('tx','rx'):
                if key not in fields:raise CalibrationError('Trace missing '+key)
                bytes.fromhex(fields[key])
            rows.append({'index':index,'raw_line':matches[0],**fields})
        result={'status':'RECORDED','motion_commands_sent':False,'records':rows}
        (folder/'bus_trace.json').write_text(json.dumps(result,indent=2),encoding='utf8')
        return {'status':'RECORDED','transactions':len(rows),'file':'bus_trace.json'}
    def close(self):
        try:self.serial.close()
        finally:self.log.close()

class Calibrator:
    def __init__(self,bridge,folder,velocity=300,torque=400,discover_mechanical_range=False):
        self.b=bridge;self.folder=folder;self.velocity=velocity;self.torque=torque
        self.original={};self.tracks={};self.results={};self.endpoints=[]
        self.last_all_health=time.monotonic()
        self.commit_confirmed=False;self.commit_cleanup=None
        self.discover_mechanical_range=discover_mechanical_range
    def backup(self):
        records=[]
        for name,motor in MOTOR_IDS.items():
            raw=(self.b.backup_snapshot(motor) if hasattr(self.b,'backup_snapshot') else
                 self.b.read(motor,0,32)+self.b.read(motor,32,8)+self.b.read(motor,40,32)+self.b.read(motor,72,8))
            if word(raw[3:5])!=777 or raw[5]!=motor:raise CalibrationError(f'ID{motor}: expected STS3215 model/ID')
            if raw[40]!=0 or raw[33]!=0:raise CalibrationError(f'ID{motor}: start requires torque off and position mode')
            if not 90<=raw[62]<=126 or raw[63]>55:raise CalibrationError(f'ID{motor}: voltage/temperature unhealthy')
            position=signed(raw[56:58],15);low=word(raw[9:11]);high=word(raw[11:13])
            if (not 0<=low<high<=4095 or not 0<=position<=4095
                or (not self.discover_mechanical_range and not low<=position<=high)):
                raise CalibrationError(f'ID{motor}: invalid baseline limits/position')
            item={'name':name,'id':motor,'eeprom_hex':raw[:40].hex(),'ram_hex':raw[40:].hex(),
                  'homing_offset':signed(raw[31:33],11),'range_min':low,'range_max':high,
                  'position':position,'voltage':raw[62]/10.,'temperature':raw[63]}
            self.original[motor]=item;records.append(item)
        data={'source':SOURCE_URL,'transport':'Windows DEBUG UART0 -> Zynq CAL1 -> UART1 servo bus',
              'motors':records,'physical_alignment_verified':False,'geometry_verified':False}
        (self.folder/'before.json').write_text(json.dumps(data,indent=2),encoding='utf8')
        print('BACKUP saved; six STS3215 IDs, torque off, existing offset/limits recorded',flush=True)
        return data
    def torque_limit(self,motor):return min(self.torque,250) if motor in (5,6) else self.torque
    def feedback(self,motor):
        # Keep independent health samples for diagnosis. Both dedicated and
        # block reads can show anomalies during motion; do not ignore them.
        raw=(self.b.read(motor,56,6)+self.b.read(motor,62,2)
             +self.b.read(motor,64,7))
        # The USB transport owns its selected temperature rule and preserves
        # actual bytes. Zynq's existing strict temperature guard stays local.
        if hasattr(self.b,'validate_feedback'):raw=self.b.validate_feedback(motor,raw)
        pos=signed(raw[:2],15);speed=signed(raw[2:4],15)
        if (not 0<=pos<=4095 or not 90<=raw[6]<=126 or raw[9]
                or (not hasattr(self.b,'validate_feedback') and raw[7]>55)):
            raise CalibrationError(f'ID{motor}: unhealthy feedback pos={pos} V={raw[6]/10} T={raw[7]} status={raw[9]}')
        if motor in self.tracks:
            t=self.tracks[motor];delta=wrapped_delta(pos,t['last']);t['unwrapped']+=delta;t['last']=pos
            logical=t['old_pos']+(t['unwrapped']-t['raw_start'])
            old=self.original[motor]
            if (not self.discover_mechanical_range
                    and not old['range_min']-100<=logical<=old['range_max']+100):
                raise CalibrationError(f'ID{motor}: existing-range travel guard reached; not a measured mechanical boundary')
        return {'position':pos,'speed':speed,'load':signed(raw[4:6],10),
                'voltage':raw[6]/10,'temperature':raw[7],'moving':raw[10],'current_raw':signed(raw[13:15],15)}
    def prepare_holds(self):
        # Set every cap first. Each HOLD replaces its own old goal atomically
        # before this servo's goal-write auto-enable. No home move.
        for motor in range(1,7):
            self.b.byte(motor,55,0);self.b.uint(motor,16,600)
            self.b.uint(motor,48,self.torque_limit(motor));self.b.byte(motor,41,5)
            self.feedback(motor);self.b.uint(motor,46,0)
            if self.discover_mechanical_range:
                # The original OFF snapshot is already saved. Broaden software
                # limits while OFF before holding the measured current pose;
                # this does not command a home pose or change the offset.
                self.b.uint(motor,9,0);self.b.uint(motor,11,4095)
        for motor in range(1,7):self.b.hold(motor)
        print('HOLD: each axis current-position goal verified by transport; no preset home',flush=True)
    def prepare_raw(self,motor):
        self.b.byte(motor,40,0);self.b.uint(motor,46,0)
        if motor not in self.tracks:
            # This servo's velocity mode exposes raw position even while the
            # EEPROM offset is nonzero. Sample the old logical frame in mode0.
            logical=self.feedback(motor)['position'];offset=self.original[motor]['homing_offset']
            self.b.uint(motor,9,0);self.b.uint(motor,11,4095);self.b.uint(motor,31,0)
            physical=self.feedback(motor)['position']
            if abs(wrapped_delta(physical,(logical+offset)%4096))>8:
                raise CalibrationError(f'ID{motor}: homing-offset sign/convention mismatch')
            self.tracks[motor]={'old_pos':logical,'raw_start':physical,'last':physical,'unwrapped':physical}
        self.b.byte(motor,33,1)
        # This STS3215 firmware resets ACC when the operating mode changes.
        self.b.byte(motor,41,5)
        if self.b.read(motor,41,1)!=b'\x05':
            raise CalibrationError(f'ID{motor}: acceleration readback after mode1 switch')
        self.b.uint(motor,46,0)
        self.b.uint(motor,48,self.torque_limit(motor));self.b.byte(motor,40,1)
    def hold_current(self,motor):
        self.b.uint(motor,46,0)
        deadline=time.monotonic()+1.5
        while abs(self.feedback(motor)['speed'])>4:
            if time.monotonic()>deadline:raise CalibrationError(f'ID{motor}: velocity did not stop')
            self.b.wait(.08)
        self.b.byte(motor,40,0);self.b.byte(motor,33,0)
        self.b.byte(motor,41,5)
        if self.b.read(motor,41,1)!=b'\x05':
            raise CalibrationError(f'ID{motor}: acceleration readback after mode0 switch')
        self.b.uint(motor,46,0);self.b.hold(motor)
    def check_all_health(self):
        if time.monotonic()-self.last_all_health<1.:return
        for motor in range(1,7):
            self.feedback(motor)
            if self.b.read(motor,40,1)!=b'\x01':
                raise CalibrationError(f'ID{motor}: holding torque unexpectedly disabled')
        self.last_all_health=time.monotonic()
    def explore(self,name,direction,label):
        motor=MOTOR_IDS[name];self.prepare_raw(motor)
        previous=self.feedback(motor)['position'];start_position=previous;travel=0;opposite=0;stable=0
        start=time.monotonic();last_print=start
        self.b.write(motor,46,encode_signed(direction*self.velocity,15))
        print(f'EXPLORE {name} ID{motor} direction={direction:+d} {label}',flush=True)
        while True:
            self.b.wait(.08);f=self.feedback(motor);now=time.monotonic()
            delta=wrapped_delta(f['position'],previous)
            if abs(delta)>600:raise CalibrationError(f'ID{motor}: discontinuous encoder feedback')
            travel+=max(0,direction*delta);opposite+=max(0,-direction*delta)
            if travel>4300 or opposite>40 or now-start>25:
                raise CalibrationError(f'ID{motor}: travel/direction/time limit reached')
            self.check_all_health()
            stable=stable+1 if abs(delta)<=2 and abs(f['speed'])<=4 and now-start>=.4 else 0
            if now-last_print>=1:
                print(f"  ID{motor} pos={f['position']} speed={f['speed']} load={f['load']} V={f['voltage']:.1f} T={f['temperature']} travel={travel}",flush=True);last_print=now
            if stable>=3:
                endpoint=f['position'];self.hold_current(motor)
                event={'name':name,'id':motor,'direction':direction,'label':label,
                       'endpoint_zero_offset':endpoint,'start_position':start_position,
                       'expected_directional_travel':(direction*(endpoint-start_position))%4096,
                       'travel':travel,'opposite_travel':opposite,'duration_s':now-start,'feedback':f}
                self.endpoints.append(event)
                (self.folder/'endpoints.json').write_text(json.dumps(self.endpoints,indent=2),encoding='utf8')
                print(f'LIMIT ID{motor} raw={endpoint} elapsed={now-start:.2f}s',flush=True)
                return endpoint,travel
            previous=f['position']
    def midpoint(self,name,aux):
        if not aux['vendor_midpoint_motion']:return
        motor=MOTOR_IDS[name];target=aux['physical_zero'];self.prepare_raw(motor)
        start=time.monotonic();previous=self.feedback(motor)['position'];remaining=(target-previous)%4096
        if remaining>aux['span']+10:raise CalibrationError('Midpoint route lies outside measured joint range')
        if remaining>3:
            self.b.write(motor,46,encode_signed(self.velocity,15))
            moved=0;opposite=0
            while moved<remaining:
                self.b.wait(.05);current=self.feedback(motor)['position'];delta=wrapped_delta(current,previous)
                self.check_all_health()
                opposite+=max(0,-delta)
                if opposite>40 or abs(delta)>600 or time.monotonic()-start>20:raise CalibrationError('Midpoint motion did not progress correctly')
                moved+=delta;previous=current
        self.hold_current(motor)
    def run(self):
        if self.discover_mechanical_range:
            print('MECHANICAL RANGE discovery: previous EEPROM soft limits are references; torque/direction/travel/time guards remain',flush=True)
        self.prepare_holds()
        for name in ORDER:
            for action,direction in PRE_ACTIONS.get(name,()):self.explore(action,direction,'pre-action')
            first,d1=self.explore(name,DIRECTIONS[name][0],'first boundary')
            second,d2=self.explore(name,DIRECTIONS[name][1],'second boundary')
            result,aux=compute_calibration(name,MOTOR_IDS[name],first,second)
            expected=self.original[MOTOR_IDS[name]]['range_max']-self.original[MOTOR_IDS[name]]['range_min']
            second_event=self.endpoints[-1]
            if (aux['span']<.75*expected or d2<.75*aux['span']
                or abs(d2-second_event['expected_directional_travel'])>30):
                raise CalibrationError(name+': measured range too short/inconsistent; possible under-torque stall')
            # Repeat the first endpoint. A one-way stall is not accepted as calibration.
            again,d3=self.explore(name,DIRECTIONS[name][0],'repeat first boundary')
            repeat_event=self.endpoints[-1]
            if (abs(wrapped_delta(again,first))>15 or d3<.75*aux['span']
                or abs(d3-repeat_event['expected_directional_travel'])>30):
                raise CalibrationError(name+': boundary repeatability failed')
            # Restore second side so vendor midpoint direction has its intended start.
            second_again,_=self.explore(name,DIRECTIONS[name][1],'return second boundary')
            if abs(wrapped_delta(second_again,second))>15:raise CalibrationError(name+': second boundary repeatability failed')
            self.midpoint(name,aux)
            self.results[name]=result
            (self.folder/'partial_calibration.json').write_text(json.dumps(self.results,indent=2),encoding='utf8')
            print('MEASURED '+name+' '+json.dumps(result),flush=True)
        for name in RECOVERY_ORDER:
            for action,direction in POST_ACTIONS.get(name,()):self.explore(action,direction,'vendor recovery')
        (self.folder/'candidate_calibration.json').write_text(json.dumps(self.results,indent=2),encoding='utf8')
        # Motion is fully stopped before any final encoder convention is applied.
        for motor in range(1,7):self.b.uint(motor,46,0);self.b.byte(motor,40,0);self.b.byte(motor,33,0)
        for name in MOTOR_IDS:
            result=self.results[name];motor=result['id'];self.b.byte(motor,55,0)
            self.b.write(motor,31,encode_signed(result['homing_offset'],11))
            self.b.uint(motor,9,result['range_min']);self.b.uint(motor,11,result['range_max'])
        verified=[]
        for name in MOTOR_IDS:
            result=self.results[name];motor=result['id']
            registers=self.b.read(motor,9,32)
            current=signed(self.b.read(motor,56,2),15)
            actual={'id':motor,'homing_offset':signed(registers[22:24],11),
                    'range_min':word(registers[:2]),'range_max':word(registers[2:4]),
                    'mode':registers[24],'torque':registers[31],'position':current}
            if (any(actual[k]!=result[k] for k in ('homing_offset','range_min','range_max'))
                or actual['mode']!=0 or actual['torque']!=0
                or not result['range_min']-15<=current<=result['range_max']+15):
                raise CalibrationError(f'ID{motor}: final configuration/position readback mismatch: {actual}')
            verified.append(actual)
        (self.folder/'after_write_readback.json').write_text(json.dumps(verified,indent=2),encoding='utf8')
        # COMMIT restores original protection/max-torque/lock, retains new offset/limits.
        cleanup=self.b.finish(True)
        self.commit_confirmed=True;self.commit_cleanup=cleanup
        (self.folder/'rk7020_right_so101.json').write_text(json.dumps(self.results,indent=2),encoding='utf8')
        print('COMMIT '+ ' | '.join(cleanup),flush=True)
        return self.results

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='COM16');parser.add_argument('--out',type=Path)
    parser.add_argument('--inspect',action='store_true',help='Backup and check bridge, no torque enable or motion')
    parser.add_argument('--velocity',type=int,default=300);parser.add_argument('--torque',type=int,default=400)
    args=parser.parse_args(argv)
    if not 50<=args.velocity<=300 or not 100<=args.torque<=400:parser.error('Velocity 50..300, torque 100..400 required')
    stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    folder=args.out or Path(__file__).resolve().parents[1]/'results'/stamp
    if folder.exists():parser.error('Output folder already exists; preserve old backups')
    folder.mkdir(parents=True)
    bridge=None;runner=None;report={'status':'NOT_COMPLETED','port':args.port,'velocity':args.velocity,'torque':args.torque,
                        'source':SOURCE_URL,'real_calibration_complete':False,
                        'physical_alignment_verified':False,'geometry_verified':False,'events':[]}
    try:
        bridge=Bridge(args.port,folder);bridge.start();runner=Calibrator(bridge,folder,args.velocity,args.torque)
        runner.backup()
        if args.inspect:
            report['cleanup']=bridge.finish(True);report['status']='INSPECT_PASS'
        else:
            report['calibration']=runner.run();report['status']='CALIBRATION_PASS';report['real_calibration_complete']=True
        report['endpoint_records']=runner.endpoints
    except BaseException as error:
        report['error']=repr(error)
        print('CALIBRATION STOPPED: '+str(error),flush=True)
        if runner and runner.commit_confirmed:
            report['status']='COMMITTED_OUTPUT_ERROR';report['cleanup']=runner.commit_cleanup
            report['rollback_not_applicable']='New EEPROM calibration already committed; candidate_calibration.json contains the measured result'
        elif bridge:
            try:report['cleanup']=bridge.finish(False)
            except Exception as cleanup_error:report['cleanup_error']=repr(cleanup_error);print('STOP/ROLLBACK NOT CONFIRMED: '+str(cleanup_error),flush=True)
        raise
    finally:
        if runner:
            report['commit_confirmed']=runner.commit_confirmed
            report['endpoint_records']=runner.endpoints
        if bridge:
            if hasattr(bridge,'capture_trace'):
                try:report['bus_trace']=bridge.capture_trace(folder)
                except Exception as trace_error:report['bus_trace_error']=repr(trace_error)
            try:bridge.close()
            except Exception as close_error:
                report['close_error']=repr(close_error)
                print('UART close warning: '+str(close_error),flush=True)
        (folder/'summary.json').write_text(json.dumps(report,indent=2),encoding='utf8')
        print('Recorded '+str(folder),flush=True)

if __name__=='__main__':
    try:main()
    except (CalibrationError,OSError,KeyboardInterrupt,ValueError) as error:raise SystemExit(str(error))
