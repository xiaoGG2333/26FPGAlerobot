"""主机侧: 发送 POSE1 求解请求, 解析 RESULT1 + TIME1 行, 报告板上纯求解耗时。

配合 fpga/diag_fw/firmware/ 的板上打点固件使用(固件在 RESULT1 后追加 TIME1 行)。
TIME1 <seq> <board_us> 为 ik_solve 全程(200 次迭代上限内实际迭代)板内耗时,
不含串口往返。
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

DIAG_DIR = Path(__file__).resolve().parents[2] / 'sw' / 'diag'
sys.path.insert(0, str(DIAG_DIR))
from right_joycon import UART  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port', required=True)
    ap.add_argument('--count', type=int, default=12)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    uart = UART(args.port, timeout=5)
    rows = []
    try:
        uart.exchange(b'HELLO\n', 'POSE1 READY ')
        uart.arm('ARM1 STATE', 'STATE')
        q, pose = uart.state()
        for k in range(args.count):
            axis, sign = k % 3, 1 if (k // 3) % 2 == 0 else -1
            delta = [0.0] * 6
            delta[axis] = sign * 0.002
            target = [pose[i] + delta[i] for i in range(3)] + pose[3:]
            t0 = time.perf_counter()
            uart.serial.write(('POSE1 %d 0 ' % uart.seq +
                               ' '.join('%.9g' % x for x in target + [0.5]) + '\n').encode())
            uart.seq += 1
            board_us = None
            deadline = time.monotonic() + 10
            buffer = bytearray()
            while time.monotonic() < deadline:
                buffer.extend(uart.serial.read_until(b'\n', size=512))
                if not buffer.endswith(b'\n'):
                    continue
                line = buffer.decode('ascii', errors='replace').strip('\x00 \t\r\n')
                buffer.clear()
                fields = line.split()
                if fields and fields[0] == 'TIME1' and int(fields[1]) == uart.seq:
                    board_us = int(fields[2])
                    break
            local_ms = (time.perf_counter() - t0) * 1000
            rows.append({'k': k, 'axis': 'XYZ'[axis], 'sign': sign,
                         'board_us': board_us, 'local_roundtrip_ms': local_ms})
            print(f'k={k:2d} board_us={board_us} roundtrip_ms={local_ms:.2f}', flush=True)
    finally:
        uart.close()
    board = sorted(r['board_us'] for r in rows if r['board_us'] is not None)
    median = board[len(board) // 2] if board else None
    report = {'rows': rows, 'board_us_median': median,
              'pass_5ms': median is not None and median < 5000}
    (args.out / 'board_ik_time.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('结果:', args.out / 'board_ik_time.json',
          '| 判定:', 'PASS(<5ms)' if report['pass_5ms'] else 'CHECK', flush=True)


if __name__ == '__main__':
    main()
