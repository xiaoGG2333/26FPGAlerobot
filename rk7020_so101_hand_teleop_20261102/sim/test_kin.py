"""运动学自检：FK 对齐固件已知值、IK 收敛、5 轴可达性限制、限位/counts。

跑法： D:\Anaconda3\envs\hand-pi\python.exe -u sim/test_kin.py
"""

import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kin  # noqa: E402


def main():
    ok = True

    # 1) FK(HOME_Q) 对齐 right_joycon.py 实测值
    T = kin.fk(kin.HOME_Q)
    pos, rpy = T[:3, 3], kin.rpy(T)
    dp = np.abs(pos - np.array([0.2726, 0.0429, 0.2383])).max()
    dr = np.abs(np.array(rpy) - np.array([0.15, -0.10, 0.20])).max()
    print(f'[1] FK(HOME_Q) pos={np.round(pos, 5)} rpy={np.round(rpy, 5)}  偏差 {dp:.1e}m '
          f'{dr:.1e}rad -> {"OK" if dp < 2e-4 and dr < 2e-4 else "FAIL"}')
    ok &= dp < 2e-4 and dr < 2e-4

    # 2) 固件主程序的两套场景（main.c: test_iterations）
    for i, (tq, iq) in enumerate((([.2, -.3, .4, -.2, .15], [.1, -.2, .3, -.1, 0.0]),
                                  ([-.25, .2, -.35, .15, -.2], [0.0] * 5))):
        r = kin.solve(kin.fk(np.array(tq)), np.array(iq), max_steps=1000,
                      tolerance=1e-10, damping=0.01, sugihara=bool(i))
        print(f'[2] SCENARIO{i} code={r["code"]} updates={r["iterations"]:>3} '
              f'E={r["energy"]:.2e} q={np.round(r["q"], 4)}')
        ok &= r['code'] == 0

    # 3) 在线走线形态：每帧 2mm 级小步 + 上一帧热启动（pose_map 的目标构造方式）
    q = kin.HOME_Q.copy()
    p0 = kin.fk(q)[:3, 3].copy()
    ts, its, codes = [], [], []
    for k in range(150):
        s = k / 150.0
        x, y, z = p0[0] - 0.03 * s, p0[1] + 0.04 * s, p0[2] + 0.02 * math.sin(s * 6)
        Tt = kin.make_target(x, y, z, 0.15 + 0.2 * s, -0.0977,
                             math.atan2(y, x - 0.0612))     # yaw 由位置反算
        t0 = time.perf_counter()
        r = kin.solve(Tt, q, max_steps=100, tolerance=1e-8, damping=0.01)
        ts.append(time.perf_counter() - t0)
        its.append(r['iterations'])
        codes.append(r['code'])
        q = r['q']
    print(f'[3] 在线序列 150 帧：IK {1000 * np.mean(ts):.2f} ms/次 '
          f'(p95 {1000 * np.percentile(ts, 95):.2f} ms), 平均迭代 {np.mean(its):.1f}, '
          f'收敛 {codes.count(0)}/150')
    ok &= codes.count(0) == 150

    # 4) 5 轴可达性限制：位置平移但姿态不变 -> 不在可达流形上，应报 MAX_ITER
    Tt = kin.fk(kin.HOME_Q).copy()
    Tt[0, 3] += 0.002
    r = kin.solve(Tt, kin.HOME_Q, max_steps=60, tolerance=1e-8)
    print(f'[4] 姿态不变只平移 2mm：code={r["code"]} (期望 1=MAX_ITER) '
          f'pos 残差={r["pos"]:.1e} -> {"OK" if r["code"] == 1 else "FAIL"}')
    ok &= r['code'] == 1

    # 5) 关节限位与 counts 转换
    c, bad = kin.q_to_counts(kin.HOME_Q)
    bad_q = kin.HOME_Q.copy()
    bad_q[2] = 3.0
    _, bad2 = kin.q_to_counts(bad_q)
    back = kin.counts_to_q(c)
    print(f'[5] HOME_Q counts={c} 越界={bad}；q[2]=3.0 -> 越界下标={bad2} (期望 2)；'
          f'counts->q 往返 {np.abs(back - kin.HOME_Q).max():.1e} '
          f'-> {"OK" if bad is None and bad2 == 2 else "FAIL"}')
    ok &= bad is None and bad2 == 2
    print(f'    夹爪 0.0/0.5/1.0 -> counts {kin.gripper_to_counts(0.0)}/'
          f'{kin.gripper_to_counts(0.5)}/{kin.gripper_to_counts(1.0)}')

    print('\n结论：', 'ALL PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
