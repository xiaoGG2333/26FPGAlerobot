"""Bounded single-servo jog through Zynq DEBUG UART0; not the adapter USB port."""
import argparse
import re
import time
import serial
from model_calibration import request as read_reply


def request(uart, command, marker, timeout):
    uart.write((command + '\n').encode('ascii'))
    deadline = time.monotonic() + timeout
    buffer = bytearray()
    relevant = []
    found = False
    while time.monotonic() < deadline:
        ch = uart.read(1)
        if not ch:
            continue
        if ch != b'\n':
            if len(buffer) < 4096:
                buffer.extend(ch)
            continue
        line = buffer.decode('ascii', errors='replace').strip()
        buffer.clear()
        print(line, flush=True)
        if line.startswith('MOTOR1 ERROR'):
            found = True
            relevant.append(line)
        if line.startswith(marker):
            found = True
            relevant.append(line)
        if found and line == 'MOTOR1 END':
            if any(x.startswith('MOTOR1 ERROR') for x in relevant):
                raise RuntimeError('; '.join(relevant))
            return relevant[-1]
    raise TimeoutError('No complete reply; a motion command will NOT be retried')


def jog(uart, seq, motor_id, delta, timeout):
    print(f'[MOVE IN 3 SECONDS] ID{motor_id} delta={delta} counts ({delta*360/4096:+.2f} encoder deg), speed={100 if motor_id==5 else 50}. Ctrl+C cancels.',flush=True)
    time.sleep(3)
    line = request(uart, f'MOTOR1 JOG {seq} {motor_id} {delta}',
                   f'MOTOR1 DONE seq={seq} ', timeout)
    if 'result=OK ' not in line or 'torque_off=1' not in line:
        raise RuntimeError(line)
    return line


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', required=True)
    p.add_argument('--timeout', type=float, default=8)
    sub = p.add_subparsers(dest='action', required=True)
    sub.add_parser('state')
    sub.add_parser('stop')
    j = sub.add_parser('jog')
    j.add_argument('id', type=int, choices=range(1, 7))
    j.add_argument('delta', type=int, help='signed encoder counts, ID5 magnitude 10..240, other IDs 10..100')
    j.add_argument('--return', dest='return_back', action='store_true',
                   help='after success, jog back to the measured original count')
    a = p.parse_args()
    if a.timeout < 6:
        p.error('timeout must be at least 6 seconds')
    maximum=240 if a.action=='jog' and a.id==5 else 100
    if a.action == 'jog' and not 10 <= abs(a.delta) <= maximum:
        p.error(f'delta magnitude must be 10..{maximum} counts')
    with serial.Serial(a.port, 115200, timeout=.1, write_timeout=2) as uart:
        # No flush/tcdrain: it can hang with VMware USB pass-through.
        try:
            if a.action == 'stop':
                line = request(uart, 'MOTOR1 STOP', 'MOTOR1 STOP ', a.timeout)
                if 'ALL_TORQUE_OFF' not in line:
                    raise RuntimeError(line)
                return
            line = request(uart, 'MOTOR1 STATE', 'MOTOR1 STATE ', a.timeout)
            if a.action == 'state':
                return
            if 'active=0 ' not in line:
                raise RuntimeError('Motion already active; send stop first')
            hello=read_reply(uart,'HELLO','POSE1 READY')
            if not any('TORQUE_SETTLE_150MS_V1' in x.split() for x in hello):
                raise RuntimeError('Update SD: TORQUE_SETTLE_150MS_V1 firmware required before jog')
            seq = int(re.search(r'last_seq=(\d+)', line).group(1)) + 1
            result = jog(uart, seq, a.id, a.delta, a.timeout)
            if a.return_back:
                origin = int(re.search(r'origin=(-?\d+)', result).group(1))
                position = int(re.search(r'position=(-?\d+)', result).group(1))
                correction = origin - position
                if 10 <= abs(correction) <= maximum:
                    jog(uart, seq + 1, a.id, correction, a.timeout)
                else:
                    raise RuntimeError(f'Return delta {correction} is outside the permitted return range; no return sent')
        except (KeyboardInterrupt, OSError, RuntimeError, TimeoutError):
            # No retry of an uncertain jog. Request torque off while the port is open.
            try:
                request(uart, 'MOTOR1 STOP', 'MOTOR1 STOP ', a.timeout)
            except Exception as stop_error:
                print('STOP could not be confirmed:', stop_error, flush=True)
            raise


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit('Interrupted; inspect the STOP reply for torque-off confirmation.')
    except (OSError, RuntimeError, TimeoutError, serial.SerialException) as error:
        raise SystemExit(str(error))
