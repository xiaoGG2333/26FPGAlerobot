"""对固件副本做板上 IK 耗时打点(幂等)。

不改原仓库;对 fpga/diag_fw/firmware/ 下的副本做两处文本替换:
  1. pose_uart.inc 顶部补 #include "xtime_l.h"
  2. ik_solve 调用前后加 XTime_GetTime,RESULT1 之后追加 TIME1 <seq> <board_us> 行
TIME1 为新增行,RESULT1 帧格式不变(主机 parse_result 不受影响)。
"""
import sys
from pathlib import Path

FIRMWARE = Path(__file__).resolve().parent / 'firmware'
POSE = FIRMWARE / 'pose_uart.inc'

INC_OLD = '/* Full ARM iteration; verified arm mode queues bounded servo trajectories. */\n#include <stdlib.h>'
INC_NEW = ('/* Full ARM iteration; verified arm mode queues bounded servo trajectories. */\n'
           '#include "xtime_l.h" /* W1 D1 board timing patch */\n'
           '#include <stdlib.h>')

T0_OLD = '    ik_result result;int selected=(int)core;\n    int code=ik_solve(target,pose_q,200,1e-8,.01f,1,arm_step,NULL,&selected,&result);'
T0_NEW = ('    XTime ik_t0,ik_t1;XTime_GetTime(&ik_t0); /* board timing patch */\n'
          '    ik_result result;int selected=(int)core;\n'
          '    int code=ik_solve(target,pose_q,200,1e-8,.01f,1,arm_step,NULL,&selected,&result);\n'
          '    XTime_GetTime(&ik_t1);')

TAIL_OLD = ('        (double)result.q[0],(double)result.q[1],(double)result.q[2],(double)result.q[3],(double)result.q[4],(double)v[6]);\n'
            '    send_text(reply);\n}')
TAIL_NEW = ('        (double)result.q[0],(double)result.q[1],(double)result.q[2],(double)result.q[3],(double)result.q[4],(double)v[6]);\n'
            '    send_text(reply);\n'
            '    {char tline[80];snprintf(tline,sizeof tline,"TIME1 %lu %llu\\r\\n",seq,\n'
            '        (unsigned long long)(((ik_t1-ik_t0)*1000000ULL)/COUNTS_PER_SECOND));send_text(tline);}\n}')


def main():
    text = POSE.read_text(encoding='utf-8')
    applied = []
    for name, old, new, marker in (
            ('include', INC_OLD, INC_NEW, '#include "xtime_l.h"'),
            ('t0', T0_OLD, T0_NEW, 'XTime ik_t0'),
            ('tail', TAIL_OLD, TAIL_NEW, 'TIME1 %lu')):
        if marker in text:
            print(f'[SKIP] {name}: 打点已存在')
            continue
        if old not in text:
            print(f'[ERROR] {name}: 目标文本未找到(固件版本不符?)')
            return 1
        text = text.replace(old, new, 1)
        applied.append(name)
    if not applied:
        print('无改动(全部已应用)')
        return 0
    POSE.write_text(text, encoding='utf-8')
    print('已应用:', ', '.join(applied))
    print('核对: xtime_l.h=%s, ik_t0=%s, TIME1=%s' % (
        'xtime_l.h' in text, 'ik_t0' in text, 'TIME1 ' in text))
    return 0


if __name__ == '__main__':
    sys.exit(main())
