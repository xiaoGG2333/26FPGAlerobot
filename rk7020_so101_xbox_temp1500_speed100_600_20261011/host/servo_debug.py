"""Query UART1 servos through Zynq DEBUG UART0. Never sends motion commands."""
import argparse
import time
import serial

def transact(uart,command,timeout):
    uart.write((command+'\n').encode('ascii'))
    deadline=time.monotonic()+timeout
    buffer=bytearray()
    while time.monotonic()<deadline:
        ch=uart.read(1)
        if not ch:
            continue
        if ch==b'\n':
            line=buffer.decode('ascii',errors='replace').strip();buffer.clear()
            print(line,flush=True)
            if line=='SERVO1 END':return
        elif len(buffer)<4096:
            buffer.extend(ch)
    raise TimeoutError('No SERVO1 END: check NEW BOOT.BIN, UART0 connection and other serial programs')

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port',required=True)
    p.add_argument('--timeout',type=float,default=15)
    p.add_argument('--baud',type=int,choices=[1000000,500000,250000,115200],default=1000000,
                   help='UART1 servo bus baud; DEBUG UART0 stays 115200')
    p.add_argument('--count',type=int,default=1)
    sub=p.add_subparsers(dest='action',required=True)
    sub.add_parser('info');sub.add_parser('scan');sub.add_parser('loopback')
    sub.add_parser('map');sub.add_parser('angles');sub.add_parser('calcheck')
    for name in ('ping','read','off'):
        s=sub.add_parser(name);s.add_argument('id',type=int,choices=range(1,254))
    a=p.parse_args()
    if a.timeout<=0 or a.count<1:p.error('timeout and count must be positive')
    with serial.Serial(a.port,115200,timeout=.1,write_timeout=2) as uart:
        # Do not flush(): VMware USB pass-through can block tcdrain().
        transact(uart,'SERVO1 BAUD '+str(a.baud),a.timeout)
        for i in range(a.count):
            command='SERVO1 '+a.action.upper()
            if hasattr(a,'id'):command+=' '+str(a.id)
            transact(uart,command,a.timeout)
            if i+1<a.count:time.sleep(.2)

if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('Stopped. No motion commands were sent.')
    except (OSError,serial.SerialException,TimeoutError) as e:raise SystemExit(str(e))
