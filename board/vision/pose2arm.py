# -*- coding: utf-8 -*-
"""阶段 0：视觉 → POSE1 → rk7100 板端（手套遥控器）。

一条链路：摄像头 → mediapipe → Geometry → PoseMapper → 安全包络 → POSE1 → 板端 IK。
不改固件、不改 HLS。视觉侧的映射、平滑、标定全部复用 sw/pose_map.py。

用法（hand-pi 环境）：
    python board/vision/pose2arm.py --port COM18 --arm            # 真动（TEST_MANUAL_ENABLE）
    python board/vision/pose2arm.py --port COM18                  # 只解 IK 不使能（预言机，臂不动）

安全约定：
- 目标先过安全包络（--env-*），再过上游的可达球夹紧（calib.reach）
- 手丢失 -> hold 上一帧目标并继续按 --hz 发送（否则 500 ms 看门狗会冻结板端）
- 连续 40 帧 IK 不收敛 / 连续 10 条 ERR1 -> ARM1 FREEZE（保力矩、停运动）
- 退出/异常 -> ARM1 STOP（关力矩）
- Ctrl+C 也走 finally
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2] / "src" / "vision"
sys.path.insert(0, str(ROOT / 'sw'))

import numpy as np                                            # noqa: E402
import cv2                                                    # noqa: E402

import pose_map as pm                                         # noqa: E402
from pose_link import BoardLink                               # noqa: E402
sys.path.insert(0, str(ROOT.parents[1] / 'sim' / 'vision'))
import kin                                                    # noqa: E402

OUT_DIR = ROOT / 'results' / 'w3_pose'


def arm_diag(text):
    """从 ARM1 回包里取 actual= 的五个 counts，报哪个关节越界（借 sim/kin 的关节表）。"""
    import re as _re
    m = _re.search(r'actual=(-?\d+,-?\d+,-?\d+,-?\d+,-?\d+)', text or '')
    if not m:
        return ''
    c = [int(v) for v in m.group(1).split(',')]
    q = kin.counts_to_q(c)
    lines = []
    for i, j in enumerate(kin.JOINTS):
        bad = []
        if c[i] < j['cmin']:
            bad.append('counts< %d' % j['cmin'])
        if c[i] > j['cmax']:
            bad.append('counts> %d' % j['cmax'])
        if q[i] < j['q_min'] - 1e-9:
            bad.append('q< %.4f' % j['q_min'])
        if q[i] > j['q_max'] + 1e-9:
            bad.append('q> %.4f' % j['q_max'])
        if bad:
            lines.append('     ID%d %-14s counts %4d  q %+.3f  <- %s'
                         % (j['id'], j['name'], c[i], q[i], ', '.join(bad)))
    if not lines:
        lines.append('     （五轴都在范围内，但板端仍拒绝：可能是对齐/几何检查）')
    return '\n'.join(lines)


def deg2cnt(deg):
    """舵机角度 -> counts（STS3215：360° 对应 4096）。"""
    return int(round(max(0.0, min(360.0, float(deg))) * 4096.0 / 360.0))


def clamp_env(pose, env):
    """把目标夹进安全包络；夹完按新位置重算 yaw（方位角约束）。"""
    x = min(max(pose['x'], env['x'][0]), env['x'][1])
    y = min(max(pose['y'], env['y'][0]), env['y'][1])
    z = min(max(pose['z'], env['z'][0]), env['z'][1])
    r = min(max(pose['roll'], -env['roll']), env['roll'])
    hit = (x, y, z, r) != (pose['x'], pose['y'], pose['z'], pose['roll'])
    pose = dict(pose, x=x, y=y, z=z, roll=r)
    pose['yaw'] = math.atan2(y, x - pm.SHOULDER_X)
    return pose, hit


def cam_source(arg):
    """auto -> 按设备名避开内置摄像头（序号会随插拔平移）。"""
    if str(arg).strip().lower() != 'auto':
        return arg
    cams = pm.list_cameras()
    if not cams:
        return None
    for idx, nm, w, h, b in cams:
        print(f'[摄像头] {idx}: {nm:<16} {"内置" if pm._looks_builtin(nm) else "外接?"} {w}x{h} {b}')
    pick = pm.pick_auto(cams)
    sel = [c for c in cams if c[0] == pick][0]
    print(f'[摄像头] auto -> {pick}（{sel[1]}）')
    if pm._looks_builtin(sel[1]):
        print('[警告] 没找到外接摄像头，现在用的是内置摄像头')
    return str(pick)


def draw_hand(vis, res, g, W, H):
    """画手部骨架（pm.draw 里那段搬出来，HUD 自己负责）。"""
    if g is None or not res.hand_landmarks:
        return
    pts = [(int(l.x * W), int(l.y * H)) for l in res.hand_landmarks[0]]
    for a, b in pm.HAND_LINKS:
        cv2.line(vis, pts[a], pts[b], (0, 200, 255), 2, cv2.LINE_AA)
    for i, (px, py) in enumerate(pts):
        col = (0, 0, 255) if i == 0 else ((80, 220, 80) if i in pm.TIP else (255, 255, 255))
        cv2.circle(vis, (px, py), 4 if i in pm.TIP else 3, col, -1)


_TEXT_CACHE = {}


def _render_texts(texts, fs, S=3):
    """把整块字先按 3 倍尺寸渲染再降采样，边缘干净、没有描边重影。

    之前用 cv2.putText 直接画两遍（粗黑底 + 细彩心），0.55 字号下
    黑底比笔画还宽，看起来就是重影；放大画再缩回来才是真抗锯齿。
    """
    key = (tuple(t for t, _ in texts), tuple(c for _, c in texts), round(fs, 3))
    hit = _TEXT_CACHE.get(key)
    if hit is not None:
        return hit
    FONT = cv2.FONT_HERSHEY_SIMPLEX
    th = S                                    # 3x 画布上 3 px -> 缩回约 1 px
    sizes = [cv2.getTextSize(t, FONT, fs * S, th) for t, _ in texts]
    W = max(s[0][0] for s in sizes) + 6 * S
    H = sum(s[0][1] + s[1] for s in sizes) + 4 * S
    col = np.zeros((H, W, 3), np.uint8)
    msk = np.zeros((H, W), np.uint8)
    y = 2 * S
    for (t, c), (sz, base) in zip(texts, sizes):
        y += sz[1]
        org = (3 * S, y)
        cv2.putText(col, t, org, FONT, fs * S, (0, 0, 0), th + 2, cv2.LINE_AA)
        cv2.putText(col, t, org, FONT, fs * S, c, th, cv2.LINE_AA)
        cv2.putText(msk, t, org, FONT, fs * S, 255, th + 2, cv2.LINE_AA)
        y += base + S
    col = cv2.resize(col, (W // S, H // S), interpolation=cv2.INTER_AREA)
    msk = cv2.resize(msk, (W // S, H // S), interpolation=cv2.INTER_AREA)
    if len(_TEXT_CACHE) > 64:
        _TEXT_CACHE.clear()
    _TEXT_CACHE[key] = (col, msk)
    return col, msk


def blit_text(img, texts, corner='tl', pad=8, fs=0.55):
    """按角位贴一组带黑描边的字，不铺底板。"""
    col, msk = _render_texts(texts, fs)
    h, w = msk.shape
    x = pad if corner[1] == 'l' else max(0, img.shape[1] - w - pad)
    y = pad if corner[0] == 't' else max(0, img.shape[0] - h - pad)
    roi = img[y:y + h, x:x + w]
    a = (msk.astype(np.float32) / 255.0)[..., None]
    roi[:] = (roi.astype(np.float32) * (1.0 - a)
              + col.astype(np.float32) * a).astype(np.uint8)
    return w, h


def draw_min_hud(vis, pose, sx, sy, sz, size_ratio, g, fps, W, H,
                 corner='tl', frozen=False):
    """极简叠层：骨架之外只有两行字，和 sw/pose_view.py 同一套数值定义。

    相对 X/Y/Z = pose 与 arm_start.xyz 的差（mm）；ROLL = pose['roll']（度）。
    """
    l1 = 'X%+7.1f  Y%+7.1f  Z%+7.1f mm' % ((pose['x'] - sx) * 1000.0,
                                            (pose['y'] - sy) * 1000.0,
                                            (pose['z'] - sz) * 1000.0)
    l2 = 'ROLL %+6.1f deg   size %.2f   %.0f fps' % (
        math.degrees(pose['roll']), float(size_ratio), fps)
    if g is None:
        l2 = 'NO HAND   last: ' + l2
    if frozen:
        l2 = 'FROZEN   ' + l2
    c2 = (140, 140, 140) if (g is None or frozen) else (120, 255, 120)
    blit_text(vis, [(l1, (0, 255, 255)), (l2, c2)], corner=corner, pad=8, fs=0.55)
    return vis


class CjkChip:
    """中文提示位图缓存：同一句话只渲染一次（PIL 每帧渲染太慢）。"""

    def __init__(self, painter, size=20, fg=(0, 0, 0)):
        self.painter, self.size, self.fg = painter, size, fg
        self.cache = {}

    def get(self, text):
        c = self.cache.get(text)
        if c is not None:
            return c
        w = int(self.painter.measure(text, self.size)) + 12
        h = int(self.size * 1.6) + 6
        img = np.full((h, w, 3), 255, np.uint8)
        img = self.painter.flush(img, [(text, (6, 3), self.fg, self.size)])
        mask = img.astype(np.int16).sum(axis=2) < 700
        if len(self.cache) > 48:
            self.cache.clear()
        self.cache[text] = (img, mask)
        return img, mask


class Hud:
    """调试叠层：左上=计算输出，左下=三个灵敏度；统一 20px 黑字。

    中文标签/单位/按键提示只渲染一次，做成静态层；数值每帧 cv2.putText。
    """

    FS = 20
    LINE = 28
    PAD = 8

    def __init__(self, W, H, painter):
        self.W, self.H, self.painter = int(W), int(H), painter
        self.fg = (0, 0, 0)
        h1 = cv2.getTextSize('0', cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0][1]
        self.sc = self.FS / float(h1)                    # 数字高度也做成 20px
        self.th = 2
        self.top = [('相对 X', 'mm'), ('相对 Y', 'mm'), ('相对 Z', 'mm'),
                    ('手部大小', ''), ('手腕旋转', chr(176)),
                    ('夹爪', ''), ('IK', 'mm')]
        self.bot = [('前后灵敏度', 'q -  w +'),
                    ('左右灵敏度', 'a -  s +'),
                    ('上下灵敏度', 'z -  x +'),
                    ('位置平滑', 't -  y +'),
                    ('旋转平滑', 'g -  h +')]
        self.hint = ['握拳2次 → 张掌 = 自动标零    c 采零位',
                     'p 存盘    r 快照    ESC 退出']
        self._build()

    def _m(self, t):
        return int(self.painter.measure(t, self.FS))

    def _build(self):
        P, FS, LINE = self.PAD, self.FS, self.LINE
        lw_top = max(self._m(t) for t, _ in self.top)
        lw_bot = max(self._m(t) for t, _ in self.bot)
        self.xv_top = P * 2 + lw_top + 104                # 数值右对齐列
        self.xv_bot = P * 2 + lw_bot + 62
        self.top_w = self.xv_top + max(self._m(u) for _, u in self.top) + 16
        self.bot_w = max(self.xv_bot + 12 + max(self._m(h) for _, h in self.bot),
                         max(self._m(x) for x in self.hint) + P * 2) + 10
        self.top_h = len(self.top) * LINE + P * 2
        self.bot_h = (len(self.hint) + len(self.bot)) * LINE + P * 2
        self.top_y0 = P
        self.bot_y0 = self.H - self.bot_h - 4
        self.rect_top = (0, self.top_y0, self.top_w, self.top_y0 + self.top_h)
        self.rect_bot = (0, self.bot_y0, self.bot_w, self.bot_y0 + self.bot_h)
        # 静态文字层（白底渲染，再用"非白"当掩码贴到画面上）
        txt = []
        for i, (lab, unit) in enumerate(self.top):
            y = self.top_y0 + P + i * LINE
            txt.append((lab, (P * 2, y), self.fg, FS))
            if unit:
                txt.append((unit, (self.xv_top + 7, y), self.fg, FS))
        for j, h in enumerate(self.hint):
            txt.append((h, (P * 2, self.bot_y0 + P + j * LINE), self.fg, FS))
        for k, (lab, keys) in enumerate(self.bot):
            y = self.bot_y0 + P + (len(self.hint) + k) * LINE
            txt.append((lab, (P * 2, y), self.fg, FS))
            txt.append((keys, (self.xv_bot + 12, y), self.fg, FS))
        layer = np.full((self.H, self.W, 3), 255, np.uint8)
        self.stat = self.painter.flush(layer, txt)
        self.mask = self.stat.astype(np.int16).sum(axis=2) < 748

    def _right(self, vis, txt, xr, y):
        w = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, self.sc, self.th)[0][0]
        cv2.putText(vis, txt, (int(xr - w), int(y + self.FS)),
                    cv2.FONT_HERSHEY_SIMPLEX, self.sc, self.fg, self.th, cv2.LINE_AA)

    def draw(self, vis, top_vals, bot_vals, flash=''):
        for (x0, y0, x1, y1) in (self.rect_top, self.rect_bot):
            roi = vis[y0:y1, x0:x1]
            cv2.addWeighted(roi, 0.28, np.full_like(roi, 255), 0.72, 0, roi)
        vis[self.mask] = self.stat[self.mask]
        for i, v in enumerate(top_vals):
            self._right(vis, v, self.xv_top, self.top_y0 + self.PAD + i * self.LINE)
        for k, v in enumerate(bot_vals):
            self._right(vis, v, self.xv_bot,
                        self.bot_y0 + self.PAD + (len(self.hint) + k) * self.LINE)
        if flash:
            img, m = self._chip.get(flash)
            y = self.bot_y0 - img.shape[0] - 6
            if y > 0:
                vis[y:y + img.shape[0], 8:8 + img.shape[1]][m] = img[m]

    def set_chip(self, chip):
        self._chip = chip


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM18', help='板子调试串口（FTDI 那个）')
    ap.add_argument('--arm', action='store_true',
                    help='真的使能机械臂（TEST_MANUAL_ENABLE）；不加则只解 IK，臂不动')
    ap.add_argument('--hz', type=float, default=20.0, help='POSE1 发送频率（115200 下别超 30）')
    ap.add_argument('--core', type=int, default=0, choices=[0, 1])
    ap.add_argument('--source', default='auto')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--model', default=str(pm.DEFAULT_MODEL))
    ap.add_argument('--calib', default=str(pm.DEFAULT_CALIB))
    ap.add_argument('--env-x', default='0.18,0.35', help='x 允许区间 m')
    ap.add_argument('--env-y', default='-0.15,0.15', help='y 允许区间 m')
    ap.add_argument('--env-z', default='0.18,0.36', help='z 允许区间 m')
    ap.add_argument('--env-roll', type=float, default=0.90, help='|roll| 上限 rad')
    ap.add_argument('--fwd-measure', default='palmrow',
                    choices=['palmrow', 'palm', 'palmw', 'world'],
                    help='前后用哪个尺度：palmrow=腕到四个掌指关节中心（默认，不含手指）；'
                         'palm=腕到中指根 p9（原算法）；palmw=食指根到小指根；'
                         'world=palmrow 再乘米制掌长比值')
    ap.add_argument('--roll-offset-deg', type=float, default=0.0,
                    help='爪子初始朝向整体偏转角度（度，正负决定顺/逆时针）；'
                         '叠加在包络夹紧之后，所以视觉 roll 量程不受影响')
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--no-window', action='store_true')
    ap.add_argument('--hud', default='full', choices=['full', 'min'],
                    help='full=原来那块中文面板；min=只留骨架+两行数值，几乎不挡画面')
    ap.add_argument('--hud-corner', default='tl', choices=['tl', 'tr', 'bl', 'br'],
                    help='min 模式下数值的位置')
    ap.add_argument('--hud-scale', type=float, default=1.0,
                    help='min 模式显示放大倍数（只放大窗口，不影响坐标）')
    ap.add_argument('--log', default=None, help='逐帧 jsonl（默认 results/w3_pose/arm_run_<ts>.jsonl）')
    ap.add_argument('--print-every', type=int, default=20)
    ap.add_argument('--seq0', type=int, default=100000,
                    help='起始序号（板端要求严格递增；RESET1 后从 seq0+1 开始）')
    ap.add_argument('--amax-scale', type=float, default=1.0,
                    help='板端轨迹 amax 相对固件默认的倍数（RAM only，0 或 1 = 不改）')
    ap.add_argument('--jmax-scale', type=float, default=1.0,
                    help='板端轨迹 jmax 相对固件默认的倍数（RAM only）')
    ap.add_argument('--wait-arm', type=float, default=90.0,
                    help='CHECK 不过时等多少秒（等人工回掰，0=不等）')
    ap.add_argument('--blend', type=float, default=6.0,
                    help='使能后从实测位姿过渡到视觉目标的秒数（0=不渡）')
    ap.add_argument('--env-auto', dest='env_auto', action='store_true',
                    help='包络中心跟实测位姿（默认关：臂可能被手动摆到任意位置，会让可动区变窄）')
    ap.add_argument('--grip', dest='grip', action='store_true', default=True,
                    help='跟臂一起启用夹爪 ID6（方向式：张掌=OPEN、握拳=CLOSE、其他=HOLD）')
    ap.add_argument('--no-grip', dest='grip', action='store_false', help='不碰夹爪')
    ap.add_argument('--grip-cap', type=int, default=300, help='夹爪扭矩上限 1~500（先温柔）')
    ap.add_argument('--grip-speed', type=int, default=100,
                    help='夹爪舵机速度 1~100，固件 st_set_goal/grip_config 硬卡 100（已是上限）')
    ap.add_argument('--grip-rate', type=int, default=400,
                    help='夹爪行程率 counts/s。实测固件硬上限 400（送 2000 会 CONFIG rc=44），'
                         '这一项就是固件能给的最快（走完全行程 716 counts 约 1.8 s）')
    ap.add_argument('--grip-closed', type=int, default=2055,
                    help='闭合端点 counts（2055=180.6°，2026-11-01 验证过的值）')
    ap.add_argument('--grip-opened', type=int, default=2771,
                    help='张开端点 counts（2771=243.5°，同上）')
    ap.add_argument('--grip-closed-deg', type=float, default=5.0,
                    help='闭合位置（度）——仅在 --grip-closed=-1 时生效')
    ap.add_argument('--grip-opened-deg', type=float, default=60.0,
                    help='张开位置（度）——仅在 --grip-opened=-1 时生效')
    ap.add_argument('--grip-ka', type=float, default=0.2,
                    help='夹爪 INPUT 保活间隔秒（固件 500 ms 不走就冻结）')
    ap.add_argument('--env-half', default='0.06,0.13,0.10,0.8',
                    help='自适应包络半宽 x,y,z,roll（米/弧度）')
    ap.add_argument('--profile', action='store_true', help='打印分项耗时')
    args = ap.parse_args()

    env = {'x': tuple(float(v) for v in args.env_x.split(',')),
           'y': tuple(float(v) for v in args.env_y.split(',')),
           'z': tuple(float(v) for v in args.env_z.split(',')),
           'roll': abs(args.env_roll)}

    def env_auto(seedp):
        """把包络中心跟到实测末端：手自然行程约 ±(0.3 u/v)*0.6 = ±0.18 m，
        取半宽后夹到绝对安全区（地面/可达球之外一律不让去）。"""
        hx, hy, hz, hr = (float(v) for v in args.env_half.split(','))
        e = {
            'x': (max(0.12, seedp['x'] - hx), min(0.40, seedp['x'] + hx)),
            'y': (max(-0.24, seedp['y'] - hy), min(0.24, seedp['y'] + hy)),
            'z': (max(0.15, seedp['z'] - hz), min(0.45, seedp['z'] + hz)),
            'roll': hr,
        }
        print('[包络] 以实测位姿为心: x %.3f~%.3f  y %.3f~%.3f  z %.3f~%.3f  |roll|<=%.2f'
              % (e['x'][0], e['x'][1], e['y'][0], e['y'][1],
                 e['z'][0], e['z'][1], e['roll']))
        return e

    # ---------------- 1. 串口与板端状态 ----------------
    link = BoardLink(args.port).open()
    print(f'[串口] {args.port} 115200 8N1')
    ok, text = link.hello()
    if not ok:
        print('[错误] HELLO 不匹配，先排查 BOOT.BIN/bit/app 是否同一套：')
        print('       ' + text[:400])
        link.close()
        return 2
    print('[板] 版本标记全部匹配')

    st = link.state()
    print(f'[板] active={st.get("active")} mode={st.get("mode")} '
          f'aligned={st.get("aligned")} geometry={st.get("geometry")} '
          f'torque_off={st.get("torque_off")}')
    if st.get('active') == '1':
        print('[中止] 板子已处于使能状态，先 `ARM1 STOP` 再运行')
        link.close()
        return 3

    armed = False
    seed = None
    grip_on, grip_dir_last, grip_t = False, None, 0.0
    win_gone = 0

    # ---------------- 2. 视觉侧 ----------------
    calib = pm.load_calib(args.calib, quiet=False)
    mapper = pm.PoseMapper(calib)
    az = pm.AutoZero(calib.get('auto_zero', {}))
    if az.enabled:
        print(f'[自动标定] 快速握拳 {az.n} 次后张开手掌 -> 采零位（{az.avg_n} 帧平均）')

    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions

    model_path = Path(args.model)
    if not model_path.exists():
        print(f'[错误] 模型不存在: {model_path}')
        link.stop()
        link.close()
        return 6

    src = cam_source(args.source)
    if src is None:
        print('[错误] 没找到摄像头')
        link.stop()
        link.close()
        return 7

    win = 'SO-101 pose2arm (q 退出 / 空格冻结)'
    if not args.no_window:
        try:
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(win, np.zeros((8, 8, 3), np.uint8))
            cv2.waitKey(1)
        except cv2.error as e:
            print('[错误] opencv 没有 GUI 后端:', str(e)[:160])
            link.stop()
            link.close()
            return 8

    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.VIDEO, num_hands=1,
        min_hand_detection_confidence=0.5, min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5)
    landmarker = vision.HandLandmarker.create_from_options(opts)
    cap = pm.open_capture(str(src), args.width, args.height, args.fps)
    if cap is None or not cap.isOpened():
        print(f'[错误] 打不开视频源 {src}')
        pm.close_quietly(landmarker)
        link.stop()
        link.close()
        return 9
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
    painter = pm.make_painter()
    print(f'[采集] {W}x{H} 源={src} | 文字 {"中文" if painter.cjk else "ASCII"}')
    hud = Hud(W, H, painter)
    hud.set_chip(CjkChip(painter))

    log_path = Path(args.log) if args.log else \
        OUT_DIR / f'arm_run_{time.strftime("%Y%m%d_%H%M%S")}.jsonl'
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = open(log_path, 'w', encoding='utf-8')
    print(f'[日志] {log_path}')

    # 板端轨迹限制（必在 active=0 时发；RAM only，断电恢复固件默认）
    if args.amax_scale not in (0.0, 1.0) or args.jmax_scale not in (0.0, 1.0):
        _ba = [1200, 1200, 1500, 1800, 1800]
        _bj = [6000, 6000, 7500, 9000, 9000]
        print('[板] 调轨迹限制 amax x%.2f jmax x%.2f（RAM only）'
              % (args.amax_scale, args.jmax_scale))
        for _i in range(5):
            _am = int(min(10000, max(1, round(_ba[_i] * args.amax_scale))))
            _jm = int(min(100000, max(1, round(_bj[_i] * args.jmax_scale))))
            _rep = ''
            for _try in range(3):
                _rep = ' '.join(link.arm('ARM1 PROFILE %d 2000 %d %d'
                                         % (_i + 1, _am, _jm)))
                if 'PROFILE OK' in _rep:
                    break
            print('     ID%d vmax 2000 amax %d jmax %d  %s'
                  % (_i + 1, _am, _jm,
                     'OK' if 'PROFILE OK' in _rep else _rep[:60]))

    # 使能放到视觉栈就绪之后：使能后 500 ms 内必须有 POSE1/KEEP，否则板端看门狗冻结
    if args.arm:
        link.keep()
        cok, ctext = link.check(manual=True)
        if not cok:
            print('[预检] TEST_MANUAL_CHECK 未通过，先看看哪个关节越界：')
            print(arm_diag(ctext))
            if args.wait_arm > 0:
                print('[预检] 力矩是关的，请手动把臂掰回限位内；最多等 %.0f s…' % args.wait_arm)
                t_wait = time.monotonic()
                while not cok and time.monotonic() - t_wait < args.wait_arm:
                    link.keep()
                    time.sleep(2.0)
                    cok, ctext = link.check(manual=True)
                    if not cok:
                        print(arm_diag(ctext))
        if not cok:
            print('[中止] TEST_MANUAL_CHECK 仍未通过，不动臂：')
            print('       ' + ctext[:300])
            link.stop()
            link.close()
            return 4
        print('[板] CHECK_OK')
        eok, etext = link.enable(manual=True)
        if not eok:
            print('[中止] TEST_MANUAL_ENABLE 失败：' + etext[:300])
            link.stop()
            link.close()
            return 5
        st = link.state()
        print(f'[板] 已使能 active={st.get("active")} mode={st.get("mode")} '
              f'counts={st.get("counts")}')
        armed = True
        # 使能成功后立刻开始喂狗：夹爪 CONFIG/ENABLE、state1、过渡准备期间也绝不超时
        link.keepalive_start(0.2)
        link.keep()
        t1 = link.state1()
        if len(t1) >= 12:
            q = [float(v) for v in t1[2:7]]
            seed_raw = {'x': float(t1[7]), 'y': float(t1[8]), 'z': float(t1[9]),
                        'roll': float(t1[10])}
            home = getattr(pm, 'HOME_Q', [0.2, -0.3, 0.4, -0.2, 0.15])
            d = max(abs(q[i] - home[i]) for i in range(5))
            print('[实测] q = [%s]  与 HOME 最大差 %.3f rad (%.0f°)' % (
                ', '.join('%.3f' % v for v in q), d, math.degrees(d)))
            print('[实测] 当前末端 x %.4f y %.4f z %.4f roll %.3f' % (
                seed_raw['x'], seed_raw['y'], seed_raw['z'], seed_raw['roll']))
            seed = seed_raw
            print(f'[过渡] 从实测位姿 {args.blend:.1f}s 线性过渡到视觉目标')
            if args.env_auto:
                env = env_auto(seed_raw)
        link.keep()
        # 夹爪：POSE1 的 gripper 字段不会写舵机，必须走独立的 GRIP1 命令（方向式）
        if args.grip:
            gc = args.grip_closed if args.grip_closed >= 0 else deg2cnt(args.grip_closed_deg)
            go = args.grip_opened if args.grip_opened >= 0 else deg2cnt(args.grip_opened_deg)
            print('[夹爪] 端点 closed=%d(%.1f°) opened=%d(%.1f°) speed=%d rate=%d cap=%d'
                  % (gc, gc * 360.0 / 4096.0, go, go * 360.0 / 4096.0,
                     args.grip_speed, args.grip_rate, args.grip_cap))
            if abs(gc - go) < 64:
                print('[夹爪] 两端点相差 <64 counts，固件会拒（GP_CONFIG）-> 本次不碰夹爪')
                okg0, d = False, {'rc': 'LOCAL'}
            else:
                okg0, d = link.grip_config(gc, go, args.grip_rate,
                                           args.grip_speed, args.grip_cap)
            if not okg0:
                print('[夹爪] CONFIG 失败 rc=%s -> 本次不碰夹爪' % d.get('rc'))
            else:
                okg0, d2 = link.grip_enable()
                if not okg0:
                    print('[夹爪] ENABLE 失败 rc=%s -> 本次不碰夹爪' % d2.get('rc'))
                else:
                    grip_on = True
                    print('[夹爪] 已使能 actual=%s goal=%s cap=%d '
                          '（张掌=开 / 握拳=合 / 其余=保持）'
                          % (d2.get('actual'), d2.get('goal'), args.grip_cap))
    else:
        print('[模式] 未使能：POSE1 只解 IK，臂不会动（预言机）')
        link.reset(link.last_sequence() + 1)
        armed = False
    last = link.last_sequence()
    seq_start = max(args.seq0, last + 1)
    print(f'[序号] 板端最后 seq={last} -> 本会话从 {seq_start} 开始')

    # ---------------- 3. 主循环 ----------------
    seq, n, last_ts_ms = seq_start, 0, -1
    t0 = time.monotonic()          # 全程用同一时钟，别混 time.time()
    next_send = t0
    interval = 1.0 / max(1.0, args.hz)
    ROLL_OFF = math.radians(args.roll_offset_deg)
    if ROLL_OFF:
        print('[roi] 爪子朝向偏置 %+.0f°（%.3f rad）' % (args.roll_offset_deg, ROLL_OFF))
    L_REF = [float(calib['zero'].get('L_world0', 0.0))]
    SIZE_DBG = [None, None]  # [0]=(palm,palmrow,palmw) [1]=(补偿后, n_ext, w)
    MEAS_SET = [False]       # 只在首次探测帧对齐一次基准（采零位换掉 zero 后不再重复覆盖）
    _FG_PALM, _FG_ROW, _FG_PW = (float(calib['zero'].get('fist_gain_palm', 0.852)),
                                 float(calib['zero'].get('fist_gain_row', 0.877)),
                                 float(calib['zero'].get('fist_gain_pw', 0.930)))
    print('[前后] 握拳补偿 增益 palm=%.3f palmrow=%.3f palmw=%.3f（<1 表示握拳时估小了）'
          % (_FG_PALM, _FG_ROW, _FG_PW))
    if L_REF[0] > 0:
        print('[前后] 沿用米制掌长基准 L_ref=%.4f m' % L_REF[0])
    fps_disp = 0.0
    freeze = False
    bad_run, err_run = 0, 0
    resume_tried = 0
    grace_t = 0.0
    abort = ''
    pose_cmd = None
    line = 'POSE1 (未开始)'
    st_board = {'code': None, 'iters': 0, 'pos': 0.0, 'rot': 0.0, 'sent': 0}
    env_hits, last_g, hd, hscore = 0, None, None, 0.0
    blend_t0 = None
    zero_buf = None
    ZERO_NEED = 12
    read_fail = 0
    prof = {'read': 0.0, 'detect': 0.0, 'map': 0.0, 'send': 0.0, 'board': 0.0,
            'draw': 0.0, 'show': 0.0}

    def tick(key, t_start):
        prof[key] += time.monotonic() - t_start

    try:
        while True:
            _t = time.monotonic()
            ok, frame = cap.read()
            tick('read', _t)
            if not ok:
                read_fail += 1                       # DSHOW 偶发丢帧，别当成流结束
                if read_fail <= 10:                  # 最多 0.1s，不许拖过 500ms 看门狗
                    time.sleep(0.01)
                    continue
                print('[结束] 视频源连续读失败')
                break
            read_fail = 0
            _t = time.monotonic()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            now = time.monotonic()
            ts_ms = max(last_ts_ms + 1, int((now - t0) * 1000))
            last_ts_ms = ts_ms
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB,
                              data=np.ascontiguousarray(rgb))
            res = landmarker.detect_for_video(mp_img, ts_ms)
            tick('detect', _t)
            dt = 1.0 / args.fps if mapper.last_t is None else (now - mapper.last_t)
            mapper.last_t = now
            n += 1

            g = None
            if res.hand_landmarks:
                k = 0
                if res.handedness:
                    hd = res.handedness[k][0].category_name
                    hscore = float(res.handedness[k][0].score)
                    if calib['handedness']['swap']:
                        hd = 'Left' if hd == 'Right' else 'Right'
                    g = pm.Geometry.compute(res.hand_landmarks[k],
                                            res.hand_world_landmarks[k],
                                            calib['extend']['margin_ratio'],
                                            1.0 if hd == 'Right' else -1.0)
                    if g is not None:
                        # ---- 前后用的尺度：全部是“不含手指”的掌部量 ----
                        #   palm   : 腕 p0 -> 中指根 p9（原来的算法）
                        #   palmrow: 腕 p0 -> 四个掌指关节 p5/p9/p13/p17 的中心（默认）
                        #   palmw  : 食指根 p5 -> 小指根 p17
                        lm = res.hand_landmarks[k]
                        _p0 = (lm[0].x, lm[0].y)
                        s_palm = math.dist(_p0, (lm[9].x, lm[9].y))
                        s_row = math.dist(_p0, ((lm[5].x + lm[9].x + lm[13].x + lm[17].x) / 4.0,
                                                (lm[5].y + lm[9].y + lm[13].y + lm[17].y) / 4.0))
                        s_pw = math.dist((lm[5].x, lm[5].y), (lm[17].x, lm[17].y))
                        SIZE_DBG[0] = (s_palm, s_row, s_pw)
                        new_px = {'palmrow': s_row, 'palm': s_palm,
                                  'palmw': s_pw, 'world': s_row}[args.fwd_measure]
                        # ---- 握拳补偿：握得越紧，MediaPipe 把掌部估得越小 ----
                        #   实测(20261103 会话)：palm 0.852 / palmrow 0.877 /
                        #   palmw 0.930（握拳/张掌 中位比值），按 n_ext 线性插值除掉
                        _ext = int(g.get('n_ext', 0))
                        _w = max(0.0, min(1.0, (4.0 - _ext) / 4.0))
                        _fg = {'palmrow': _FG_ROW, 'world': _FG_ROW,
                               'palm': _FG_PALM, 'palmw': _FG_PW}[args.fwd_measure]
                        new_px = new_px / (1.0 - (1.0 - _fg) * _w)
                        SIZE_DBG[1] = (new_px, _ext, _w)
                        if not MEAS_SET[0]:
                            MEAS_SET[0] = True
                            calib['zero']['hand_px0'] = new_px
                            calib['zero']['size_measure'] = args.fwd_measure
                            print('[前后] 尺度 %s 基准=%.4f（首次探测帧），'
                                  '重新采零位后以采零结果为准' % (args.fwd_measure, new_px))
                        g['hand_px'] = new_px
                        lw = res.hand_world_landmarks[k]
                        Lw = math.sqrt((lw[9].x - lw[0].x) ** 2
                                       + (lw[9].y - lw[0].y) ** 2
                                       + (lw[9].z - lw[0].z) ** 2)
                        if Lw > 1e-6:
                            if L_REF[0] <= 0.0:
                                L_REF[0] = Lw
                                calib['zero']['L_world0'] = Lw
                                print('[前后] 米制掌长基准 L_ref=%.4f m' % Lw)
                            if args.fwd_measure == 'world':
                                g['hand_px'] = g['hand_px'] * (L_REF[0] / Lw)

            if g is not None:
                if zero_buf is not None:
                    zero_buf.append(g)
                    if len(zero_buf) >= ZERO_NEED:
                        z = pm.apply_zero(calib, pm._avg_geo(zero_buf))
                        mapper.reset_smoothing()
                        zero_buf = None
                        print(f'  [采零位] {ZERO_NEED} 帧平均 -> u {z["u"]:.4f} v {z["v"]:.4f} '
                              f'手长 {z["hand_px0"] * W:.1f}px theta {z["theta_deg"]:+.1f}° （w 存盘）')
                if float(calib['zero'].get('hand_px0', 0.0)) <= 1e-6:
                    hp = float(g['hand_px'])
                    if pm.MIN_HAND_PX < hp < 0.60:
                        calib['zero']['hand_px0'] = hp
                        print(f'[前后] hand_px0 缺失，用当前帧手长 {hp * W:.1f}px 作基准')
                pose, flags = mapper.map(g, dt)
                g['palm_deg'] = pose.get('palm_deg', 0.0)
                last_g = g
                auto_g = az.feed(g, now)
                if auto_g is not None:
                    z = pm.apply_zero(calib, auto_g)
                    mapper.reset_smoothing()
                    print(f'[自动标定] 零位已更新 u {z["u"]:.4f} v {z["v"]:.4f} '
                          f'手长 {z["hand_px0"] * W:.1f}px theta {z["theta_deg"]:+.1f}°')
            else:
                az.feed(None, now)
                pose, flags = mapper.hold(), ['丢手保持']

            pose, hit = clamp_env(pose, env)
            if hit:
                env_hits += 1
                flags = list(flags) + ['包络']
            if ROLL_OFF != 0.0:
                # 爪子朝向整体偏转：叠在包络之后，视觉 roll 量程不变；
                # 再夹到固件 q4 允许范围 [-2.9437, +2.8486]，否则板端会拒
                pose = dict(pose, roll=max(-2.84, min(2.84, pose['roll'] + ROLL_OFF)),
                            yaw=pose['yaw'])
                flags = list(flags) + ['roll偏置']

            # 从实测位姿平滑渡到视觉目标，避免一上来就甩到包络里
            if seed is not None and args.blend > 0:
                if blend_t0 is None:
                    blend_t0 = now
                w = min(1.0, (now - blend_t0) / args.blend)
                if w < 1.0:
                    for k in ('x', 'y', 'z', 'roll'):
                        pose[k] = seed[k] + (pose[k] - seed[k]) * w
                    pose['yaw'] = math.atan2(pose['y'], pose['x'] - pm.SHOULDER_X)
                    flags = list(flags) + ['过渡']
                else:
                    seed = None

            if freeze and pose_cmd is not None:
                pose, flags = pose_cmd, ['冻结']
            tick('map', _t)

            # 按固定频率发送；不发送就等于不喂狗，500 ms 后板子自己冻结
            if now >= next_send:
                seq += 1
                _t = time.monotonic()
                line = pm.encode_pose1(seq, args.core, pose)
                try:
                    link.pose(seq, pose, args.core)
                except Exception as e:
                    abort = f'串口写失败: {e}'
                    break
                st_board['sent'] += 1
                next_send += interval
                if now - next_send > 0.5:            # 卡顿太久就别补发，重新对时
                    next_send = now + interval
                log.write(json.dumps({
                    'ts_ms': ts_ms, 'seq': seq, 'detected': g is not None,
                    'handedness': hd, 'flags': flags, 'env_hit': hit, 'held': g is None,
                    'hand_px': None if not g else round(float(g['hand_px']) * W, 1),
                    'raw': None if not getattr(mapper, 'dbg', None) else {
                        k: round(float(v), 4) for k, v in getattr(mapper, 'dbg', {}).items()
                        if k in ('u', 'v', 'd_roll_deg')},
                    'size_ratio': round(mapper.size_ratio, 4),
                    'sizes': None if not SIZE_DBG[0] else {
                        'palm': round(SIZE_DBG[0][0], 4),
                        'palmrow': round(SIZE_DBG[0][1], 4),
                        'palmw': round(SIZE_DBG[0][2], 4),
                        'fix': None if not SIZE_DBG[1] else round(SIZE_DBG[1][0], 4),
                        'next': None if not SIZE_DBG[1] else SIZE_DBG[1][1]},
                    'pose1': {k: round(float(v), 6) for k, v in pose.items()},
                    'pose1_line': line,
                }, ensure_ascii=False) + '\n')
                pose_cmd = pose
                if grip_on:
                    gv = float(pose.get('gripper', 0.5))
                    nd = 'OPEN' if gv >= 0.75 else 'CLOSE' if gv <= 0.25 else 'HOLD'
                    if nd != grip_dir_last:
                        link.grip_set(nd)          # 只写不等回包，保住 20Hz
                        grip_dir_last = nd
                        grip_t = now
                tick('send', _t)

            # 收板端回包
            _t = time.monotonic()
            err_seen = link.counts['err1']
            for r in link.drain():
                st_board.update(code=r['code'], iters=r['iters'], pos=r['pos'], rot=r['rot'])
                log.write(json.dumps({'ts_ms': ts_ms, 'result': r}, ensure_ascii=False) + '\n')
                if r['code'] == 0:
                    bad_run, err_run = 0, 0
                else:
                    bad_run += 1
            if link.counts['err1'] > err_seen:
                d_err = link.counts['err1'] - err_seen
                if now >= grace_t:                 # 恢复宽限期内不计入中止阈值
                    err_run += d_err
                le = link.last_err
                if le and 'ARM_FROZEN' in le[1] and resume_tried < 3:
                    resume_tried += 1
                    print('[板] 检测到 ARM_FROZEN（看门狗），第 %d 次尝试 ARM1 RESUME'
                          % resume_tried)
                    print('     ' + ' '.join(link.resume())[:160])
                    err_run = 0
                    grace_t = now + 1.0
                    if grip_on:                     # 冻结会把夹爪使能一起打掉
                        okg2, d2 = link.grip_enable()
                        print('     [夹爪] 重新 ENABLE rc=%s' % d2.get('rc'))
                        link._grip_on = bool(okg2)
            tick('board', _t)

            if bad_run >= 40:
                abort = f'连续 {bad_run} 帧 IK 不收敛'
                link.freeze()
                break
            if err_run >= 10:
                abort = f'连续/累计 {err_run} 条 ERR1'
                link.freeze()
                break

            # ---------------- 画面 ----------------
            _t = time.monotonic()
            vis = frame.copy()
            draw_hand(vis, res, g, W, H)
            _sx, _sy, _sz = [float(v) for v in calib['arm_start']['xyz']]
            if n % 10 == 0 or n == 1:
                fps_disp = n / max(1e-6, time.monotonic() - t0)
            _flash = '已冻结（继续发上一帧目标）' if freeze else (
                az.flash if now < az.flash_until else '')
            if args.hud == 'min':
                draw_min_hud(vis, pose, _sx, _sy, _sz, mapper.size_ratio, g,
                             fps_disp, W, H, corner=args.hud_corner, frozen=freeze)
                if _flash:
                    blit_text(vis, [(_flash, (0, 220, 255))], corner='bl', pad=8,
                              fs=0.55)
            else:
                top_vals = [
                    '%+.1f' % ((pose['x'] - _sx) * 1000.0),
                    '%+.1f' % ((pose['y'] - _sy) * 1000.0),
                    '%+.1f' % ((pose['z'] - _sz) * 1000.0),
                    '%.3f' % mapper.size_ratio,
                    '%+.1f' % math.degrees(pose['roll']),
                    '%.2f %s' % (float(pose.get('gripper', 0.5)), grip_dir_last or '-'),
                    '%s / %s / %.3f' % (
                        '-' if st_board['code'] is None else st_board['code'],
                        '-' if st_board['iters'] is None else st_board['iters'],
                        (st_board['pos'] or 0.0) * 1000.0),
                ]
                bot_vals = ['%.2f' % calib['pos']['gain_x_size'],
                            '%.2f' % calib['pos']['gain_y'],
                            '%.2f' % calib['pos']['gain_z_v'],
                            '%.2f' % calib['smooth']['ema_pos'],
                            '%.2f' % calib['smooth']['ema_angle']]
                hud.draw(vis, top_vals, bot_vals, flash=_flash)

            tick('draw', _t)
            if not args.no_window:
                _t = time.monotonic()
                if args.hud_scale != 1.0:
                    vis = cv2.resize(vis, None, fx=args.hud_scale, fy=args.hud_scale,
                                     interpolation=cv2.INTER_NEAREST)
                cv2.imshow(win, vis)
                k = cv2.waitKey(1) & 0xFF
                tick('show', _t)
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    # 最小化时 OpenCV 也会返回 <1，要连续多帧才算真关
                    win_gone += 1
                    if win_gone >= 15:
                        print('  窗口已关闭')
                        break
                else:
                    win_gone = 0
                if k == 27:                       # ESC 退出（q 键改为调灵敏度）
                    break
                elif k == ord(' '):
                    freeze = not freeze
                    print('  冻结' if freeze else '  解冻（重新发送）')
                elif k == ord('r'):
                    ts = time.strftime('%H%M%S')
                    (OUT_DIR / 'snapshots').mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(OUT_DIR / 'snapshots' / f'arm_{ts}.png'), vis)
                    print(f'  快照 -> snapshots/arm_{ts}.png')
                elif k == ord('c'):
                    if last_g is None:
                        print('  没检到手，先把手放进画面再按 c')
                    else:
                        zero_buf = []
                        print(f'  采零位：保持不动 {ZERO_NEED} 帧…')
                elif k == ord('p'):
                    pm.save_calib(calib, args.calib)
                    print('  calib 已存盘')
                elif k in (ord('q'), ord('w'), ord('a'), ord('s'),
                           ord('z'), ord('x')):
                    pz = calib['pos']
                    if k in (ord('q'), ord('w')):
                        pz['gain_x_size'] = min(1.50, max(0.05, pz['gain_x_size']
                                                        + (-0.05 if k == ord('q') else 0.05)))
                        print('  [前后灵敏度] %.2f' % pz['gain_x_size'])
                    elif k in (ord('a'), ord('s')):
                        pz['gain_y'] = min(1.50, max(0.10, pz['gain_y']
                                                     + (-0.05 if k == ord('a') else 0.05)))
                        print('  [左右灵敏度] %.2f' % pz['gain_y'])
                    else:
                        pz['gain_z_v'] = min(1.50, max(0.10, pz['gain_z_v']
                                                       + (-0.05 if k == ord('z') else 0.05)))
                        print('  [上下灵敏度] %.2f' % pz['gain_z_v'])
                elif k in (ord('t'), ord('y')):
                    sm = calib['smooth']
                    sm['ema_pos'] = min(0.95, max(0.20, sm['ema_pos']
                                                  + (-0.05 if k == ord('t') else 0.05)))
                    print('  [位置平滑] ema_pos %.2f  (越小越稳但越滞后)' % sm['ema_pos'])
                elif k in (ord('g'), ord('h')):
                    sm = calib['smooth']
                    sm['ema_angle'] = min(0.95, max(0.20, sm['ema_angle']
                                                    + (-0.05 if k == ord('g') else 0.05)))
                    print('  [旋转平滑] ema_angle %.2f  (越小越稳但越滞后)'
                          % sm['ema_angle'])
            if not args.no_window and n % max(1, args.print_every) == 0:
                d = getattr(mapper, 'dbg', {}) or {}
                ch = ''
                if d:
                    ch = ('  u %.3f v %.3f sz %.3f th %+.1f'
                          % (d.get('u', 0), d.get('v', 0), mapper.size_ratio,
                             d.get('theta_deg', 0)))
                gp = f'  grip {grip_dir_last}' if grip_on else ''
                print(f'[{seq:05d}] {line}  code {st_board["code"]} '
                      f'it {st_board["iters"]} pos {st_board["pos"] * 1000:.3f}mm{ch}{gp}'
                      + (f'  [{",".join(flags)}]' if flags else ''))
                if args.profile and n:
                    print('   耗时/帧 ' + ' '.join('%s %.1f' % (k, v * 1000 / n)
                                                  for k, v in prof.items()) + ' ms')
                    for k in prof:
                        prof[k] = 0.0
            if args.max_frames and n >= args.max_frames:
                break
    except KeyboardInterrupt:
        abort = 'Ctrl+C'
    finally:
        print(f'\n[退出] {abort or "正常"}')
        try:
            link.keepalive_stop()
        except Exception:
            pass
        if grip_on:
            try:
                link.grip_stop()
                print('[夹爪] STOP')
            except Exception as e:
                print(f'[夹爪] STOP 失败: {e}')
        try:
            got = link.stop()
            print('[板] STOP: ' + ' '.join(got)[:200])
        except Exception as e:
            print(f'[板] STOP 失败: {e}')
        link.close()
        log.close()
        cap.release()
        pm.close_quietly(landmarker)
        if not args.no_window:
            cv2.destroyAllWindows()

        el = max(1e-6, time.monotonic() - t0)
        print(f'[统计] {n} 帧 / {el:.1f}s = {n / el:.1f} fps | '
              f'发送 {link.counts["pose"]} 帧 POSE1 | 回包 {link.counts["result"]} '
              f'(code0 {link.counts["code0"]}) | ERR1 {link.counts["err1"]} | '
              f'包络夹紧 {env_hits} 帧')
        summary = {'frames': n, 'seconds': round(el, 1), 'fps': round(n / el, 1),
                   'armed': armed, 'hz': args.hz, 'port': args.port,
                   'sent': link.counts['pose'], 'result': link.counts['result'],
                   'code0': link.counts['code0'], 'err1': link.counts['err1'],
                   'env_hits': env_hits, 'abort': abort, 'log': str(log_path),
                   'calib': args.calib}
        (OUT_DIR / f'arm_session_{time.strftime("%Y%m%d_%H%M%S")}.json').write_text(
            json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
    return 0


if __name__ == '__main__':
    try:
        _code = main()
    except BaseException as _e:            # 任何异常都必须先把力矩关掉再走
        if isinstance(_e, SystemExit) and _e.code == 0:
            from pose_link import _LIVE
            if _LIVE is None:
                raise
        import traceback
        traceback.print_exc()
        try:
            import pose_link as _pl
            if _pl._LIVE is not None and getattr(_pl._LIVE, 'ser', None) is not None:
                print('[兜底] 异常退出 -> ARM1 STOP')
                _pl._LIVE.stop()
                _pl._LIVE.close()
        except Exception as _e2:
            print(f'[兜底] STOP 也失败了: {_e2} —— 请手动断电或按板子 STOP')
        _code = 1
    # mediapipe 1.1.1 的 C++ 调度线程退出时会卡 ~40 s，硬退
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(_code if isinstance(_code, int) else 0)
