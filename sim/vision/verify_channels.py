"""视觉 5 通道 -> POSE1 -> 逆解核 的接口体检（纯数值：不碰摄像头、不开 GUI）

用法（hand-pi 环境）：
    python sim/verify_channels.py            # 扫 5 个通道：增益 / 线性区间 / 饱和原因 / 串扰
    python sim/verify_channels.py --rate     # 随机人手工况 360 帧：逐帧增量 / IK 收敛 / 行合法性

结论文档：docs/接口-视觉5通道-到逆解核-确认.md
结果落盘：results/sim/trace5_channels.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "src" / "vision"
for _p in (Path(__file__).resolve().parent, ROOT / 'sw'):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
import kin                      # noqa: E402
import pose_map                 # noqa: E402

KEYS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'gripper')


def load():
    calib = pose_map.load_calib(ROOT / 'results' / 'w3_pose' / 'calib.json', quiet=True)
    if float(calib['zero'].get('hand_px0', 0.0)) <= 0:
        calib['zero']['hand_px0'] = 0.16       # 现场是 0（没采过），测试用一个典型值
        print('[注意] calib.zero.hand_px0 = 0，本次测试临时用 0.16 代替（真机必须先采到）')
    return calib


def geo(calib, dtheta=0.0, u=None, v=None, ratio=None, n_ext=5):
    z = calib['zero']
    px0 = float(z['hand_px0'])
    return dict(theta=math.radians(z['theta_deg'] + dtheta),
                phi=math.radians(z['phi_deg']), psi=math.radians(z['psi_deg']),
                twist=math.radians(z.get('twist_deg', 0.0)),
                palm=(z['u'] if u is None else u, z['v'] if v is None else v),
                palm_deg=z.get('palm_deg', 0.0), point=math.radians(z.get('point_deg', 0.0)),
                cuff=math.radians(z.get('cuff_deg', 0.0)),
                cuff2d=math.radians(z.get('cuff2d_deg', 0.0)),
                pal_r=float(z.get('palm_r0', 0.86)), pal_cos=1.0, pal_sin=0.0,
                hand_px=px0 if ratio is None else px0 / float(ratio),
                n_ext=n_ext, ext=[1] * n_ext + [0] * (5 - n_ext))


def settle(calib, kw, n=900, dt=0.5):
    """喂同一输入直到收敛（dt 取上限，用来跨过 slew 限速拿稳态）。"""
    m = pose_map.PoseMapper(calib)
    for _ in range(n):
        pose, flags = m.map(geo(calib, **kw), dt)
    return pose, flags


def scan(calib, base, name, inname, outkey, samples, nominal, scale=1.0):
    pts = [(x, settle(calib, kw)[0]) for kw, x in samples]
    xs = [p[0] for p in pts]
    ys = [p[1][outkey] * scale for p in pts]
    slopes = [(ys[i + 1] - ys[i]) / (xs[i + 1] - xs[i]) for i in range(len(xs) - 1)]
    lin = [i for i, s in enumerate(slopes) if abs(s) >= 0.9 * abs(nominal)]
    flat = [(xs[i], xs[i + 1]) for i, s in enumerate(slopes) if abs(s) < 0.1 * abs(nominal)]
    print('\n=== %s ===' % name)
    print('  输入 %s 扫 %+.3f..%+.3f（%d 点），逐段输出增量：%s' % (
        inname, xs[0], xs[-1], len(xs), ' '.join('%+.4f' % s for s in slopes)))
    if not lin:
        print('  没有找到线性段（增益全被截断）')
        return dict(name=name, slope=float('nan'))
    xx = [xs[i] for i in lin] + [xs[i + 1] for i in lin]
    yy = [ys[i] for i in lin] + [ys[i + 1] for i in lin]
    k, b = np.polyfit(xx, yy, 1)
    resid = float(np.abs(np.array(yy) - (k * np.array(xx) + b)).max())
    span = max(yy) - min(yy)
    print('  标称增益 %.4f %s/%s｜线性段 %s∈[%+.3f, %+.3f] -> %s∈[%+.4f, %+.4f]（斜率 %+.4f）' % (
        nominal, outkey, inname, inname, min(xx), max(xx), outkey, min(yy), max(yy), k))
    print('  线性残差 %.2e（%.2f%% 量程）｜%.0f%% 的步长斜率在 0.9~1.1 倍标称内' % (
        resid, 100 * resid / max(1e-12, span), 100 * len(lin) / len(slopes)))
    if flat:
        print('  平段（死区/饱和）：%s' % ' '.join('[%+.3f,%+.3f]' % e for e in flat))
    cross = {}
    for kk in KEYS:
        if kk == outkey:
            continue
        vals = [p[kk] for x, p in pts if min(xx) - 1e-9 <= x <= max(xx) + 1e-9]
        cross[kk] = max(abs(v - base[kk]) for v in vals)
    print('  线性段内其它通道最大偏移：%s' % '  '.join('%s %.2e' % (kk, v) for kk, v in cross.items()))
    return dict(name=name, slope=float(k), resid=resid, xlo=float(min(xx)), xhi=float(max(xx)),
                ylo=float(min(yy)), yhi=float(max(yy)), cross=cross)


def do_scan(calib):
    base, _ = settle(calib, {})
    print('零位: ' + '  '.join('%s%+.4f' % (k, base[k]) for k in KEYS))
    print('roll 源 %r 符号 %+g 增益 %g 死区 %.1f° 饱和 ±%.0f°' % (
        calib['roll']['source'], calib['roll']['sign'], calib['roll']['gain'],
        calib['roll']['deadzone_deg'], calib['roll']['max_offset_deg']))
    print('slew %.1f mm/s, %.1f °/s ｜ pos gain y %g z %g x %g ｜ reach margin %.0f mm' % (
        calib['slew']['max_translation_m_s'] * 1000,
        math.degrees(calib['slew']['max_rotation_rad_s']),
        calib['pos']['gain_y'], calib['pos']['gain_z_v'], calib['pos']['gain_x_size'],
        calib['reach']['margin_m'] * 1000))
    p, r = calib['pos'], calib['roll']
    out = [dict(zero=base)]
    out.append(scan(calib, base, '通道1 左右 u→y', 'u', 'y',
                    [({'u': d}, d) for d in np.arange(0.05, 0.96, 0.05)],
                    p['sign_u'] * p['gain_y']))
    out.append(scan(calib, base, '通道2 上下 v→z', 'v', 'z',
                    [({'v': d}, d) for d in np.arange(0.05, 0.96, 0.05)],
                    -p['sign_v'] * p['gain_z_v']))
    out.append(scan(calib, base, '通道3 前后 手长比→x', 'ratio', 'x',
                    [({'ratio': rr}, rr) for rr in np.arange(0.60, 1.81, 0.06)],
                    -p['sign_size'] * p['gain_x_size']))
    out.append(scan(calib, base, '通道4 旋转 Δtheta→roll', 'deg', 'roll',
                    [({'dtheta': d}, d) for d in np.arange(-110, 111, 10)],
                    r['sign'] * r['gain'], scale=180 / math.pi))
    print('\n=== 通道5 开合 n_ext→gripper ===')
    for ne in (5, 4, 3, 2, 1, 0):
        print('  n_ext=%d -> gripper %.2f' % (ne, settle(calib, {'n_ext': ne})[0]['gripper']))
    print('  规则：5→%.1f  0→%.1f  1~4→保持（hold_others=%s，无限速：Δgripper 可一帧跳到底）' % (
        calib['gripper']['open'], calib['gripper']['closed'], calib['gripper']['hold_others']))
    pth = ROOT / 'results' / 'sim' / 'trace5_channels.json'
    pth.parent.mkdir(parents=True, exist_ok=True)
    pth.write_text(json.dumps(out, ensure_ascii=False, indent=1, default=float), encoding='utf-8')
    print('\n[输出] %s' % pth.relative_to(ROOT))


def do_rate(calib, n=360):
    m = pose_map.PoseMapper(calib)
    rng = np.random.default_rng(7)
    q = np.array(kin.HOME_Q, float)
    prev, mx = None, {k: 0.0 for k in KEYS}
    ok_ik = lines_ok = viol = 0
    it = 0
    ms = res = 0.0
    for i in range(n):
        t = i / 30.0
        kw = dict(u=0.32 + 0.10 * math.sin(2 * math.pi * 0.20 * t) + rng.normal(0, 0.002),
                  v=0.80 - 0.06 * math.sin(2 * math.pi * 0.13 * t + 1.0) + rng.normal(0, 0.002),
                  ratio=1.0 + 0.10 * math.sin(2 * math.pi * 0.17 * t + 2.0),
                  dtheta=40 * math.sin(2 * math.pi * 0.25 * t),
                  n_ext=5 if (int(t * 1.5) % 2 == 0) else 0)
        pose, flags = m.map(geo(calib, **kw), 1 / 30)
        toks = pose_map.encode_pose1(i + 1, 1, pose).split()
        if toks[0] == 'POSE1' and len(toks) == 10:
            try:
                lines_ok += int(all(math.isfinite(float(x)) for x in toks[3:10]))
            except ValueError:
                pass
        if not all(pose_map.FW_LIMIT[k][0] - 1e-9 <= pose[k] <= pose_map.FW_LIMIT[k][1] + 1e-9
                   for k in KEYS):
            viol += 1
        if prev is not None:
            for k in KEYS:
                mx[k] = max(mx[k], abs(pose[k] - prev[k]))
        prev = dict(pose)
        Tt = kin.make_target(pose['x'], pose['y'], pose['z'],
                             pose['roll'], pose['pitch'], pose['yaw'])
        t0 = time.perf_counter()
        r = kin.solve(Tt, q, max_steps=100, tolerance=1e-8, damping=0.01, stall=8)
        ms += (time.perf_counter() - t0) * 1000
        it += r['iterations']
        res = max(res, r['pos'])
        if r['code'] == 0:
            ok_ik += 1
            q = np.array(r['q'], float)
    print('=== %d 帧随机人手工况（30 Hz）===' % n)
    print('  逐帧最大增量: dx %.3f dy %.3f dz %.3f mm | droll %.3f dPitch %.3f dyaw %.3f deg'
          ' | dGrip %.3f' % (mx['x'] * 1000, mx['y'] * 1000, mx['z'] * 1000,
                             math.degrees(mx['roll']), math.degrees(mx['pitch']),
                             math.degrees(mx['yaw']), mx['gripper']))
    print('  超固件硬限 %d/%d 帧 | POSE1 行合法 %d/%d' % (viol, n, lines_ok, n))
    print('  IK: 收敛 %d/%d，%.1f 步/帧，%.2f ms/帧，最大位置残差 %.3f mm' % (
        ok_ik, n, it / n, ms / n, res * 1000))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rate', action='store_true', help='跑随机人手工况逐帧体检（默认扫 5 通道）')
    ap.add_argument('--frames', type=int, default=360)
    a = ap.parse_args()
    calib = load()
    if a.rate:
        do_rate(calib, a.frames)
    else:
        do_scan(calib)
    return 0


if __name__ == '__main__':
    sys.exit(main())
