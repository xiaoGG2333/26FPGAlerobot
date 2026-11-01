"""Independent ID6 protocol; no implicit calibration or EEPROM writes."""
import json
from pathlib import Path

class GripperClient:
    def __init__(self, path):
        p=json.loads(Path(path).read_text(encoding='utf-8'))
        if p.get('verified') is not True:
            raise RuntimeError('ID6 endpoints/direction are not verified; calibrate before --gripper-control')
        names=['closed_count','open_count','rate_counts_s','servo_speed','torque_cap']
        self.values=[p[k] for k in names]
        if any(type(v) is not int for v in self.values):raise ValueError('ID6 profile values must be integers')
        closed,opened,rate,speed,cap=self.values
        if not (0<=closed<=4095 and 0<=opened<=4095 and abs(closed-opened)>=64 and 1<=rate<=400 and 1<=speed<=100 and 1<=cap<=500):
            raise ValueError('ID6 profile outside allowed bounds')
        self.state={}
    def request(self,uart,command,require_active=True):
        line=uart.exchange((command+'\n').encode('ascii'),'GRIP1 STATE ')
        values={k:int(v) for k,v in (part.split('=',1) for part in line.split()[2:])}
        self.state=values
        if values.get('rc')!=0 or (require_active and (values.get('active')!=1 or values.get('frozen')!=0 or values.get('valid')!=1)):
            raise RuntimeError('Gripper frozen or stopped: '+line)
        return values
    def start(self,uart):
        self.request(uart,'GRIP1 CONFIG '+' '.join(map(str,self.values)),False)
        return self.request(uart,'GRIP1 ENABLE')
    def update(self,uart,direction):
        if direction not in (-1,0,1):raise ValueError('Invalid gripper direction')
        return self.request(uart,'GRIP1 INPUT '+{-1:'CLOSE',0:'HOLD',1:'OPEN'}[direction])
    def resume(self,uart):return self.request(uart,'GRIP1 RESUME')
