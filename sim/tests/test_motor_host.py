"""No hardware access: exercise host framing, sequence choice and stop on errors."""
from pathlib import Path
import importlib.util
import sys
import types

root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'host'))
commands=[]
mode='normal'
class Serial:
    def __init__(self,*args,**kw):self.buffer=bytearray()
    def __enter__(self):return self
    def __exit__(self,*args):return False
    def read(self,n):
        out=bytes(self.buffer[:n]);del self.buffer[:n];return out
    def flush(self):raise AssertionError('Must not call flush/tcdrain')
    def write(self,data):
        command=data.decode().strip();commands.append(command)
        if command=='HELLO':
            reply='POSE1 READY JOG_WRIST_20DEG_V1 TORQUE_SETTLE_150MS_V1\n'
        elif command=='MOTOR1 STATE':
            reply='MOTOR1 STATE active=0 last_seq=7 id=6 position=2423 target=2423 result=OK torque_off=1\nMOTOR1 END\n'
        elif command=='MOTOR1 STOP':
            reply='MOTOR1 STOP ALL_TORQUE_OFF\nMOTOR1 END\n'
        elif mode=='error':
            reply='MOTOR1 ERROR seq=8 LIMIT\nMOTOR1 END\n'
        else:
            seq=command.split()[2]
            reply=(f'MOTOR1 ACCEPTED seq={seq} id=6 origin=2423 target=2523\n'
                   f'MOTOR1 DONE seq={seq} id=6 result=OK origin=2423 target=2523 position=2521 torque_off=1\nMOTOR1 END\n')
        self.buffer.extend(reply.encode());return len(data)
sys.modules['serial']=types.SimpleNamespace(Serial=Serial,SerialException=OSError)
spec=importlib.util.spec_from_file_location('motor_debug',root/'host/motor_debug.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
sys.argv=['motor_debug','--port','mock','jog','6','100','--return']
module.main()
assert commands==['MOTOR1 STATE','HELLO','MOTOR1 JOG 8 6 100','MOTOR1 JOG 9 6 -98'],commands
commands.clear();mode='error'
sys.argv=['motor_debug','--port','mock','jog','6','100']
try:module.main()
except RuntimeError as error:assert 'LIMIT' in str(error)
else:raise AssertionError('Rejected jog accepted')
assert commands==['MOTOR1 STATE','HELLO','MOTOR1 JOG 8 6 100','MOTOR1 STOP'],commands
print('PASS: accepted/completed framing, next sequence, measured return delta, rejected motion stops without retry, no flush')
