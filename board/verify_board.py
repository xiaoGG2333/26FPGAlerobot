"""Read-only RK7100 UART verification: no torque, goal, EEPROM, or reset commands."""
import argparse,json,time
from pathlib import Path
import serial
p=argparse.ArgumentParser()
p.add_argument('--port',required=True)
p.add_argument('--servos',action='store_true')
p.add_argument('--out',type=Path,default=Path('results/board_readonly.json'))
a=p.parse_args()
def request(uart,command,prefix,timeout=10):
    uart.write((command+'\n').encode('ascii'));uart.flush()
    lines=[];deadline=time.monotonic()+timeout
    while time.monotonic()<deadline:
        line=uart.readline().decode('ascii',errors='replace').strip()
        if line:
            print(line,flush=True);lines.append(line)
            if line.startswith(prefix):return lines
    raise TimeoutError('No matching reply: '+command)
report={'port':a.port,'motion_commands_sent':False,'torque_commands_sent':False,'eeprom_write_sent':False,'responses':{}}
with serial.Serial(a.port,115200,timeout=.1,write_timeout=2) as uart:
    hello=request(uart,'HELLO','POSE1 READY',30)
    if not any('BOARD_RK7100_F_V1' in line for line in hello):raise RuntimeError('RK7100 firmware identity missing; verify SD BOOT.BIN')
    if not any('CHIP_THERMAL1_V1' in line for line in hello):raise RuntimeError('Thermal firmware required; update BOOT.BIN')
    report['responses']['hello']=hello
    report['responses']['thermal']=request(uart,'THERMAL1','THERMAL1 ')
    report['responses']['arm']=request(uart,'ARM1 STATE','ARM1 END')
    if a.servos:
        for i in range(1,7):report['responses'][str(i)]=request(uart,'SERVO1 READ '+str(i),'SERVO1 END')
a.out.parent.mkdir(parents=True,exist_ok=True)
a.out.write_text(json.dumps(report,indent=2),encoding='utf8')
print('Read-only responses saved:',a.out)
