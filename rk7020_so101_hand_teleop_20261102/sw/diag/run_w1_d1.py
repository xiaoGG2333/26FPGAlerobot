"""W1 D1 板上基线总入口(新工程版,不改原仓库固件)。

链路: 电脑 <-> (UART0 调试串口) Zynq 固件 <-> (UART1) 舵机总线
红线遵守:
  - 全程只读反馈,不发送运动命令,不使能扭矩,不写 EEPROM
  - IK probe 为"无运动求解":目标 = 当前位姿 ±2mm 微扰,仅验证求解收敛与耗时

流程:
  1. 静止反馈诊断(ID1-6 位置/电压/温度/负载,多轮采样) -> static_<ts>/summary.json
  2. ID2 漂移统计(重点盯 ID2 保持漂移历史) -> 写入汇总
  3. 可选 --ik-probe: 无运动 IK 求解 K 次,统计往返耗时与板上耗时上界
  4. 汇总 -> results/w1_diagnostics/<ts>/w1_d1_summary.json

用法:
  python run_w1_d1.py --port COM16 --samples 20 --ik-probe --probe-count 10

判定(见输出):
  STATIC 诊断:    全 ID 电压 9~12.6V、温度 <=55C、扭矩为 0 -> 基线健康
  IK 耗时:        往返中位数 - 串口开销(~15ms @115200) < 5ms 视为板上单解通过
"""
import argparse
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

DIAG_DIR = Path(__file__).resolve().parent
RESULTS = DIAG_DIR.parents[1] / 'results' / 'w1_diagnostics'

# 串口开销估算: RESULT1 行 ~120B + POSE1 请求 ~55B @ 115200 8N1
BAUD = 115200
SERIAL_OVERHEAD_MS = ((120 + 55) * 10 / BAUD) * 1000  # 约 15.2ms


def static_diag(port, samples, out_dir):
    """复用原仓库 diagnose_feedback.py(CAL1 静止诊断, 无运动/无扭矩/无EEPROM写)。"""
    script = DIAG_DIR / 'diagnose_feedback.py'
    cmd = [sys.executable, str(script), '--port', port,
           '--samples', str(samples), '--out', str(out_dir / 'static')]
    print('[1/3] 静止反馈诊断:', ' '.join(cmd), flush=True)
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=1200)
    print(proc.stdout, end='')
    if proc.returncode != 0:
        print(proc.stderr, file=sys.stderr)
        raise RuntimeError('静止诊断失败(可能板卡未连接/端口错误/反馈不健康)')
    return out_dir / 'static' / 'summary.json'


def id2_drift(summary_path):
    """从静止诊断 summary 提取 ID2 位置序列,统计漂移。"""
    report = json.loads(summary_path.read_text(encoding='utf-8'))
    rows = [r for r in report['samples'] if r['id'] == 2]
    pos = [r['position'] for r in rows]
    drift = max(pos) - min(pos)
    return {
        'id2_samples': len(pos),
        'id2_position_min': min(pos), 'id2_position_max': max(pos),
        'id2_drift_counts': drift,
        'id2_drift_deg': drift * 360.0 / 4096.0,
        'note': '漂移基线;历史漂移 2003->2021(18 计数)。演示中避免 ID2 长时间高负载保持',
    }


def ik_probe(port, count):
    """无运动 IK 求解计时: 目标 = 当前位姿 ±2mm, 不驱动机械臂。"""
    sys.path.insert(0, str(DIAG_DIR))
    from right_joycon import UART, Mapper
    from diagnose_teleop import measured_q, servo_read

    config = json.loads((DIAG_DIR / 'right_config.json').read_text(encoding='utf-8'))
    mapping = json.loads((DIAG_DIR / 'right_mapping.json').read_text(encoding='utf-8'))
    uart = UART(port, timeout=5)
    try:
        hello = uart.exchange(b'HELLO\n', 'POSE1 READY ')
        state = uart.arm('ARM1 STATE', 'STATE')
        if 'active=0' not in state:
            raise RuntimeError('机械臂正在运动控制会话中,先停止再诊断: ' + state)
        servos = [servo_read(uart, i) for i in range(1, 6)]
        if not all(s['torque'] == 0 and s['moving'] == 0
                   and 9 <= s['voltage'] <= 12.6 and s['temp'] <= 55 for s in servos):
            raise RuntimeError('静止健康检查未通过(扭矩/电压/温度): ' + json.dumps(servos))
        counts = [s['raw_pos'] for s in servos]
        measured_q(counts, mapping)  # 与固件一致的换算预检
        q, pose = uart.state()
        check = uart.arm('ARM1 TEST_MANUAL_CHECK', 'CHECK_OK')
        if not check.startswith('ARM1 CHECK_OK '):
            raise RuntimeError('只读预检被拒: ' + check)
        probes = []
        for k in range(count):
            uart.arm('ARM1 TEST_MANUAL_CHECK', 'CHECK_OK')
            axis, sign = k % 3, 1 if (k // 3) % 2 == 0 else -1
            delta = [0.0] * 6
            delta[axis] = sign * 0.002
            target = Mapper(config, pose, [0.0] * 6).target(delta, 1.0)
            t0 = time.perf_counter()
            result = uart.solve(0, target, 0.5)
            result['_local_ms'] = (time.perf_counter() - t0) * 1000
            result['axis'] = 'XYZ'[axis]
            result['sign'] = sign
            probes.append(result)
        trips = sorted(p['_local_ms'] for p in probes)
        median = trips[len(trips) // 2]
        board_upper = median - SERIAL_OVERHEAD_MS
        converged = all(p['code'] == 0 for p in probes)
        return {
            'hello': hello, 'state': state, 'probes': probes,
            'roundtrip_ms_median': median, 'roundtrip_ms_all': trips,
            'serial_overhead_ms_est': SERIAL_OVERHEAD_MS,
            'board_single_solve_ms_upper_bound': board_upper,
            'pass_5ms': board_upper < 5.0,
            'converged_all': converged,
            'note': '上界 = 往返中位数 - 串口开销估算;精确板上打点见 fpga/diag_fw/',
        }
    finally:
        uart.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', required=True, help='Zynq UART0 调试串口, 如 COM16')
    ap.add_argument('--samples', type=int, default=20, help='静止诊断采样轮数(1-100)')
    ap.add_argument('--ik-probe', action='store_true', help='无运动 IK 求解计时')
    ap.add_argument('--probe-count', type=int, default=10, help='IK probe 次数(6 的倍数为佳)')
    args = ap.parse_args()
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = RESULTS / ts
    out.mkdir(parents=True, exist_ok=False)
    summary = {'timestamp': ts, 'port': args.port,
               'motion_commands_sent': False, 'eeprom_write_sent': False,
               'torque_enable_sent': False}
    try:
        static_path = static_diag(args.port, args.samples, out)
        summary['static_summary'] = str(static_path.relative_to(RESULTS.parent))
        summary['id2_drift'] = id2_drift(static_path)
        if args.ik_probe:
            print(f'[2/3] 无运动 IK 求解计时 x{args.probe_count} ...', flush=True)
            summary['ik_timing'] = ik_probe(args.port, args.probe_count)
            verdict = 'PASS(板上单解上界<5ms)' if summary['ik_timing']['pass_5ms'] \
                else 'CHECK(上界>=5ms, 需 fpga/diag_fw 精确打点复核)'
            print(f'[2/3] 往返中位数 {summary["ik_timing"]["roundtrip_ms_median"]:.1f}ms, '
                  f'板上上界 {summary["ik_timing"]["board_single_solve_ms_upper_bound"]:.2f}ms -> {verdict}', flush=True)
        summary['status'] = 'DONE'
        print('[3/3] 汇总完成', flush=True)
    except BaseException as e:
        summary['status'] = 'FAIL'
        summary['error'] = repr(e)
        raise
    finally:
        (out / 'w1_d1_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        print('结果目录:', out, flush=True)


if __name__ == '__main__':
    main()
