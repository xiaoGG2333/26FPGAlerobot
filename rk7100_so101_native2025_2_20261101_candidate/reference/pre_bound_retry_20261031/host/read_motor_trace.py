"""Read frozen bus transactions after a motor test; never enable or move motors."""
import argparse,json,re
from pathlib import Path
import serial
from model_calibration import request

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args()
    if a.out.exists():p.error('Output exists; choose a new path')
    with serial.Serial(a.port,115200,timeout=.1,write_timeout=2) as u:
        state=request(u,'MOTOR1 STATE','MOTOR1 END')
        if not any(x.startswith('MOTOR1 STATE active=0 ') for x in state):
            raise RuntimeError('Motor test must be inactive before reading trace')
        lines=request(u,'CAL1 TRACE COUNT','CAL1 END')
        match=[re.fullmatch(r'CAL1 TRACE COUNT (\d+)',x) for x in lines]
        match=[x for x in match if x]
        if len(match)!=1:raise RuntimeError('No trace count reply')
        n=int(match[0][1])
        if n>1024:raise RuntimeError('Unexpected trace count')
        records=[request(u,f'CAL1 TRACE {i}','CAL1 END') for i in range(n)]
    a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps({'motor_state':state,'records':records,'motion_commands_sent':False},indent=2))
    print('Saved',a.out,'records=',n)

if __name__=='__main__':main()
