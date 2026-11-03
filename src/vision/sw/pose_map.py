"""B1 视觉走线：MediaPipe HandLandmarker(21 点) -> 几何后处理 -> POSE1 指令行。

链路：摄像头 -> HandLandmarker(VIDEO) -> 第 4 节几何(theta/phi/伸指数/掌心中心)
      -> 标定(零位/符号/增益/死区) -> POSE1 七字段 -> 固件硬范围夹紧 -> 打印 + jsonl

**不接板子**：只打印 POSE1 行并写 results/w3_pose/pose_run_<时间>.jsonl。
字段口径与固件范围见 docs/接口-视觉侧接入原摇杆控制链.md（yaw 一律由位置反算，绝不由手给）。

角度定义（docs/交接-B1-视觉走线开工.md 第 4 节，全部几何后处理、不训练）：
  theta  画面内旋转 = atan2((p9-p0).y, (p9-p0).x)  [图像 y 向下]  -> roll  -> ID5 wrist_roll
  phi    腕部俯仰   = 掌面法线绕光轴去旋转后的 x-z 倾角        -> pitch -> ID4 wrist_flex
  psi    诊断量     = 掌面法线去旋转后的 y-z 倾角（绕前臂轴 roll 的 3D 估计，仅日志/对照）
  伸指数  sum(|tip-p0| > |pip-p0| + margin)                    -> gripper (5->张开 / 0->闭合 / 其他保持)

按键（窗口内）：q/ESC 退出  s 快照  r 录像  空格 冻结  k 清平滑
  标定：c 采零位  x roll 符号取反  p pitch 符号取反  y 横向(y)符号取反
        g/G roll 增益  h/H pitch 增益  d/D roll 死区  f/F pitch 死区  w 保存 calib.json

命令行举例：
  python -u sw/pose_map.py --source 0                              # 摄像头实时
  python -u sw/pose_map.py --source 0 --no-window --max-frames 60  # 自检
  python -u sw/pose_map.py --source clip.mp4 --record rec.mp4      # 离线视频回放
"""
import argparse
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = ROOT / 'model' / 'data' / 'external' / 'mediapipe' / 'hand_landmarker.task'
DEFAULT_OUT = ROOT / 'results' / 'w3_pose'
DEFAULT_CALIB = DEFAULT_OUT / 'calib.json'

# 固件硬范围（pose_uart.inc:93-94，越界直接 ERR1 拒绝）
FW_LIMIT = {'x': (-0.5, 0.6), 'y': (-0.6, 0.6), 'z': (-0.3, 0.7),
            'roll': (-math.pi, math.pi), 'pitch': (-1.4, 1.4),
            'yaw': (-math.pi, math.pi), 'gripper': (0.0, 1.0)}
SHOULDER_X = 0.0612          # yaw 反算用的肩部原点（与 right_joycon.py 同源）
# 手长（归一化腕→中指根）小于这个值就当误检/手太远：实测内置摄像头会锁到 1～8px 的
# 假目标，拿它当零位基准或前后基准会把 size_ratio 弄爆（x 直接被顶到边界）。
MIN_HAND_PX = 0.08
# 5 轴臂真实可达球（从 shoulder_pan 轴算）：固件 joint0 原点 (TX[0],0,TZ[0])，
# 半径 = 各连杆长度之和（与 q 无关）。sim/kin.py 的 PAN_ORIGIN / REACH_FROM_PAN 同源。
PAN_ORIGIN = (SHOULDER_X, 0.0, 0.0598)
REACH_MAX = 0.3738


def clamp_reach(xyz, margin=0.030, center=PAN_ORIGIN, r_max=REACH_MAX):
    """把目标径向夹进可达球，返回 (夹紧后的 xyz, 超出量 m)。

    径向缩放不改方位角，所以 yaw = atan2(y, x-SHOULDER_X) 这条流形约束仍然成立。
    """
    d = np.asarray(xyz, dtype=float) - np.asarray(center, dtype=float)
    r = float(np.linalg.norm(d))
    lim = float(r_max) - float(margin)
    if r <= lim or r < 1e-9:
        return [float(v) for v in xyz], 0.0
    p = np.asarray(center, dtype=float) + d * (lim / r)
    return [float(p[0]), float(p[1]), float(p[2])], r - lim

# MediaPipe 21 点连接（mediapipe 1.1 已无 solutions 模块，这里自带一份）
HAND_LINKS = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8),
              (5, 9), (9, 10), (10, 11), (11, 12), (9, 13), (13, 14), (14, 15), (15, 16),
              (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]
TIP = [4, 8, 12, 16, 20]
PIP = [3, 6, 10, 14, 18]          # 拇指用 IP(3)，其余用 PIP
PALM_IDX = [0, 5, 9, 13, 17]

# HOME_Q = [0.2,-0.3,0.4,-0.2,0.15] 的 FK 结果（与 right_joycon.py 同一套 ET 模型）
ARM_START = {'xyz': [0.2726, 0.0429, 0.2383], 'roll': 0.15, 'pitch': -0.10}
WORKSPACE_MIN = [0.12, -0.30, 0.03]      # right_config.json 实测可用包络（小于固件硬限）
WORKSPACE_MAX = [0.45, 0.30, 0.45]


def default_calib():
    return {
        'version': 1,
        'note': 'B1 走线标定参数；零位用标定模式 c 键采集，w 键保存',
        'handedness': {'swap': False, 'select': 'any', 'min_score': 0.5},
        'zero': {'theta_deg': 0.0, 'phi_deg': 0.0, 'psi_deg': 0.0, 'twist_deg': 0.0,
                 'u': 0.5, 'v': 0.5, 'palm_r0': 0.86,
                 'point_deg': 0.0, 'cuff_deg': 0.0, 'cuff2d_deg': 0.0,
                 'hand_px0': 0.0},
        'auto_zero': {'enable': True, 'grasps': 2, 'window_s': 2.5,
                      'open_wait_s': 4.0, 'average_frames': 5, 'cooldown_s': 2.5,
                      'hold_frames': 2, 'gap_reset_s': 0.4, 'min_hand_px': 0.08},
        'roll': {'source': 'theta', 'sign': 1.0, 'gain': 1.0, 'deadzone_deg': 4.0,
                 'max_offset_deg': 90.0},
        'pitch': {'source': 'hold', 'sign': 1.0, 'gain': 0.6, 'deadzone_deg': 3.0,
                  'soft_limit_rad': 0.85, 'v_scale_deg': 90.0},
        'pos': {'sign_u': 1.0, 'sign_v': 1.0, 'sign_size': 1.0,
                'gain_y': 0.60, 'gain_z_v': 0.60, 'gain_x_size': 0.45,
                'deadzone_uv': 0.008, 'deadzone_size': 0.03},
        'arm_start': {'xyz': list(ARM_START['xyz']), 'roll': ARM_START['roll'],
                      'pitch': ARM_START['pitch']},
        # 可达球：工作空间盒比真实可达域大，超了固件只会 MAX_ITER，所以默认进球夹紧
        'reach': {'enable': True, 'margin_m': 0.030},
        'gripper': {'open': 1.0, 'closed': 0.0, 'hold_others': True, 'init': 0.5},
        'extend': {'margin_ratio': 0.15},
        'smooth': {'ema_angle': 0.60, 'ema_pos': 0.65, 'vote': 3},
        'slew': {'max_translation_m_s': 0.15, 'max_rotation_rad_s': 0.45},
    }


def deep_merge(base, over):
    out = dict(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def load_calib(path, quiet=False):
    calib = default_calib()
    p = Path(path)
    if p.exists():
        calib = deep_merge(calib, json.loads(p.read_text(encoding='utf-8')))
        if not quiet:
            print(f'[标定] 载入 {p}')
    elif not quiet:
        print(f'[标定] {p} 不存在，用默认值（按 c 采零位后按 w 保存）')
    return calib


def save_calib(calib, path):
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    calib['saved'] = time.strftime('%Y-%m-%d %H:%M:%S')
    p.write_text(json.dumps(calib, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'[标定] 已保存 -> {p}')


def wrap(v):
    return (v + math.pi) % (2 * math.pi) - math.pi


def unit(v):
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else np.zeros(3)


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def smooth_angle(prev, angle, alpha):
    """在 (cos, sin) 分量上做 EMA —— 直接平滑角度会在 ±180° 撕裂。"""
    c, s = math.cos(angle), math.sin(angle)
    if prev is None:
        return (c, s), angle
    pc, ps = prev
    c = alpha * c + (1 - alpha) * pc
    s = alpha * s + (1 - alpha) * ps
    return (c, s), math.atan2(s, c)


def apply_deadzone(v, dz):
    if abs(v) <= dz:
        return 0.0
    return math.copysign(abs(v) - dz, v)


def make_painter():
    """有中文字体就用 PIL 画中文，否则退回 ASCII(cv2.putText 不支持中文)。"""
    try:
        from PIL import Image, ImageDraw, ImageFont
        for fp in ('C:/Windows/Fonts/msyh.ttc', 'C:/Windows/Fonts/simhei.ttf'):
            if Path(fp).exists():
                cache = {}

                def get(size):
                    if size not in cache:
                        cache[size] = ImageFont.truetype(fp, size)
                    return cache[size]

                class P:
                    cjk = True

                    @staticmethod
                    def measure(t, size):
                        return get(size).getlength(t)

                    @staticmethod
                    def flush(bgr, texts):
                        img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                        d = ImageDraw.Draw(img)
                        for t, xy, col, size in texts:
                            d.text((xy[0] + 1, xy[1] + 1), t, font=get(size), fill=(0, 0, 0))
                            d.text((xy[0], xy[1]), t, font=get(size), fill=(col[2], col[1], col[0]))
                        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
                return P
    except Exception:
        pass

    class P:
        cjk = False

        @staticmethod
        def measure(t, size):
            sc = max(0.32, size / 28.0)
            return float(cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, sc, 1)[0][0])

        @staticmethod
        def flush(bgr, texts):
            for t, xy, col, size in texts:
                cv2.putText(bgr, t, (int(xy[0]), int(xy[1] + size)), cv2.FONT_HERSHEY_SIMPLEX,
                            max(0.32, size / 28.0), col, 1, cv2.LINE_AA)
            return bgr
    return P


class Geometry:
    """21 点 -> 第 4 节那几个量（纯几何后处理，不训练）。"""

    @staticmethod
    def compute(lm2d, lmw, margin_ratio, hand_sign=1.0):
        p = np.array([[l.x, l.y] for l in lm2d], float)          # 归一化图像坐标（y 向下）
        w = np.array([[l.x, l.y, l.z] for l in lmw], float)      # 米制，原点=手掌几何中心
        palm = p[PALM_IDX].mean(0)

        d2 = p[9] - p[0]
        theta = math.atan2(float(d2[1]), float(d2[0]))           # 画面内旋转（图像坐标）

        # 掌面法线：先统一到"掌心朝相机"分支，再绕光轴去旋转 theta_w
        n = unit(np.cross(w[5] - w[0], w[17] - w[0]))
        if n[2] > 0:                                            # world z 正负号由标定确认
            n = -n
        theta_w = math.atan2(float(w[9][1] - w[0][1]), float(w[9][0] - w[0][0]))
        ca, sa = math.cos(-theta_w), math.sin(-theta_w)
        nx = ca * n[0] - sa * n[1]
        ny = sa * n[0] + ca * n[1]
        nz = abs(float(n[2])) + 1e-6
        phi = math.atan2(float(nx), nz)                          # 指尖方向倾斜 -> 腕俯仰
        psi = math.atan2(float(ny), nz)                          # 前臂轴倾斜（诊断）

        # 绕前臂轴的扭转角 σ（旋前/旋后）：前臂轴 ≈ 腕->中指根；在该轴的垂面上量法线的方位
        f = unit(w[9] - w[0])
        ref = np.array([1.0, 0.0, 0.0])
        a = ref - float(np.dot(ref, f)) * f
        if float(np.linalg.norm(a)) < 1e-6:
            ref = np.array([0.0, 1.0, 0.0])
            a = ref - float(np.dot(ref, f)) * f
        a = unit(a)
        b = np.cross(f, a)
        twist = math.atan2(float(np.dot(n, b)), float(np.dot(n, a)))

        # 方案6（掌朝向 alpha，全量程 ±180）：先取“掌心法线” n_p，再绕光轴去旋转
        #   实测：掌心朝镜头时 n_z·hand_sign < 0（右手 hand_sign=+1，左手 -1）
        #   故 n_p = hand_sign * n_raw 恒指向“掌心外侧”，α 才能区分 掌心/手背朝镜头
        n_raw = unit(np.cross(w[5] - w[0], w[17] - w[0]))            # 未翻转的原始法线
        n_p = n_raw * float(hand_sign)
        pnx = ca * n_p[0] - sa * n_p[1]
        pny = sa * n_p[0] + ca * n_p[1]
        pnz = float(n_p[2])
        pn_norm = max(1e-6, math.hypot(float(pny), pnz))
        pal_sin = float(pny) / pn_norm                                # sin α
        pal_cos = -pnz / pn_norm                                      # cos α（掌心朝镜头 => +1）
        hand_len = max(1e-6, float(np.linalg.norm(p[9] - p[0])))      # 腕->中指根
        pal_r = float(np.linalg.norm(p[17] - p[5])) / hand_len        # 掌宽 / 手长

        # ===== 用户定义的手部坐标系（2026-10-17）=====
        #   爪头指向 T = 腕 -> 中指根（小臂延续方向）；虎口方向 G = 食指根 - 小指根
        #   point = T 的仰角（0=水平前指，+90=指天）
        #   cuff  = 虎口绕 T 相对“向上”的滚转角（0=虎口朝上，全量程 ±180 可分掌心/手背）
        T = unit(w[9] - w[0])
        UPW = np.array([0.0, -1.0, 0.0])                     # 实测 world y 向下
        point = math.asin(max(-1.0, min(1.0, float(np.dot(T, UPW)))))
        Gv = w[5] - w[17]
        Gv = Gv - float(np.dot(Gv, T)) * T                   # 去掉沿指向分量
        Uv = UPW - float(np.dot(UPW, T)) * T                 # "上"在垂直平面内的分量
        if float(np.linalg.norm(Uv)) < 1e-6:                 # T 正上/正下：改用相机右向当参考
            e = np.array([1.0, 0.0, 0.0])
            Uv = e - float(np.dot(e, T)) * T
        if float(np.linalg.norm(Gv)) < 1e-9 or float(np.linalg.norm(Uv)) < 1e-9:
            cuff = 0.0
        else:
            Gv = unit(Gv)
            Uv = unit(Uv)
            cuff = math.atan2(float(np.dot(T, np.cross(Gv, Uv))), float(np.dot(Gv, Uv)))

        # 虎口的画面内方位：0=虎口朝上，+90=虎口朝画面右（你的姿势下最稳，不依赖深度）
        d_t = p[5] - p[17]
        cuff2d = math.atan2(float(d_t[0]), float(-d_t[1]))

        # 前臂轴与相机光轴的夹角（越小越接近退化姿势）
        f_arm = unit(w[9] - w[0])
        arm_tilt = math.degrees(math.acos(max(-1.0, min(1.0, abs(float(f_arm[2]))))))

        scale = max(1e-6, float(np.linalg.norm(p[9] - p[0])))
        margin = margin_ratio * scale
        ext = [1 if np.linalg.norm(p[t] - p[0]) > np.linalg.norm(p[pip] - p[0]) + margin else 0
               for t, pip in zip(TIP, PIP)]
        return {'palm': (float(palm[0]), float(palm[1])), 'theta': theta, 'phi': phi, 'psi': psi,
                'twist': twist, 'pal_r': pal_r, 'pal_sin': pal_sin,
                'pal_cos': pal_cos, 'arm_tilt_deg': arm_tilt,
                'point': point, 'cuff': cuff, 'cuff2d': cuff2d,
                'theta_w': theta_w, 'ext': ext, 'n_ext': int(sum(ext)),
                'normal': (float(n[0]), float(n[1]), float(n[2])), 'hand_px': scale}


class PoseMapper:
    """几何量 + 标定 -> POSE1 七字段（含 yaw 反算、双限幅、固件硬范围夹紧）。"""

    def __init__(self, calib):
        self.set_calib(calib)
        self.last_t = None

    def set_calib(self, calib):
        self.c = calib
        self.start = calib['arm_start']
        sx, sy, sz = [float(v) for v in self.start['xyz']]
        yaw0 = math.atan2(sy, sx - SHOULDER_X)
        self.prev = {'xyz': [sx, sy, sz], 'roll': float(self.start['roll']),
                     'pitch': float(self.start['pitch']), 'yaw': yaw0}
        self.prev_yaw = yaw0
        self.gripper = float(calib['gripper']['init'])
        self.reset_smoothing()

    def reset_smoothing(self):
        self.s_theta = self.s_phi = self.s_psi = self.s_twist = self.s_palm = None
        self.s_point = self.s_cuff = self.s_cuff2d = self.uv = None
        self.size_ratio = 1.0
        self.roll_off = 0.0

    def map(self, g, dt):
        c, flags = self.c, []
        a = c['smooth']['ema_angle']
        self.s_theta, theta = smooth_angle(self.s_theta, g['theta'], a)
        self.s_phi, phi = smooth_angle(self.s_phi, g['phi'], a)
        self.s_psi, psi = smooth_angle(self.s_psi, g['psi'], a)
        self.s_twist, twist = smooth_angle(self.s_twist, g.get('twist', 0.0), a)
        self.s_point, point = smooth_angle(self.s_point, g.get('point', 0.0), a)
        self.s_cuff, cuff = smooth_angle(self.s_cuff, g.get('cuff', 0.0), a)
        self.s_cuff2d, cuff2d = smooth_angle(self.s_cuff2d, g.get('cuff2d', 0.0), a)

        ap = c['smooth']['ema_pos']
        u, v = g['palm']
        self.uv = (u, v) if self.uv is None else (ap * u + (1 - ap) * self.uv[0],
                                                 ap * v + (1 - ap) * self.uv[1])
        su, sv = self.uv

        z0, rc, pc, pos = c['zero'], c['roll'], c['pitch'], c['pos']

        # 掌朝向 alpha = atan2(法线正弦分量, 掌宽比投影)  —— 方案6 正交融合
        #   sin 取自法线（正对镜头时准），cos 取自掌宽/手长的投影比（转到侧掌时准）
        r0 = float(z0.get('palm_r0', 0.86)) or 0.86
        cos_n = float(g.get('pal_cos', 1.0))                            # 法线给正负
        if g['n_ext'] >= 4:          # 只有张开手掌时宽度比才可信（握拳会改变掌宽/手长）
            cos_mag = max(0.0, min(1.0, float(g.get('pal_r', r0)) / r0))
        else:
            cos_mag = min(1.0, abs(cos_n))
        cos_a = math.copysign(cos_mag, cos_n if abs(cos_n) > 0.15 else 1.0)
        alpha_raw = math.atan2(float(g.get('pal_sin', 0.0)), cos_a)
        self.s_palm, alpha = smooth_angle(self.s_palm, alpha_raw, a)

        # ---- 位置：相对零位的手位移 -> 工作空间位移，再夹到 工作空间 ∩ 固件硬限 ----
        #   u(画面横向) -> y 左右 ； v(画面纵向) -> z 上下 ； 手的长短(远近) -> x 前后
        du = apply_deadzone(su - z0['u'], pos['deadzone_uv'])
        dv = apply_deadzone(sv - z0['v'], pos['deadzone_uv'])
        hp0 = float(z0.get('hand_px0', 0.0))
        hpx = float(g.get('hand_px', 0.0))
        if hp0 > 1e-6 and hpx > 1e-6:
            size_ratio = hp0 / hpx                      # >1 手比标定时小(更远)，<1 手更大(更近)
        else:
            size_ratio = 1.0
        dsz = apply_deadzone(size_ratio - 1.0, pos['deadzone_size'])
        self.size_ratio = hp0 / hpx if (hp0 > 1e-6 and hpx > 1e-6) else 1.0
        xyz = [self.start['xyz'][0] + pos['sign_size'] * pos['gain_x_size'] * (-dsz),
               self.start['xyz'][1] + pos['sign_u'] * pos['gain_y'] * du,
               self.start['xyz'][2] - pos['sign_v'] * pos['gain_z_v'] * dv]
        for i, k in enumerate('xyz'):
            lo = max(WORKSPACE_MIN[i], FW_LIMIT[k][0])
            hi = min(WORKSPACE_MAX[i], FW_LIMIT[k][1])
            if not lo <= xyz[i] <= hi:
                flags.append(k)
            xyz[i] = clamp(xyz[i], lo, hi)

        # ---- 可达球：工作空间盒（z 到 0.45）比真实可达域大很多，5 轴臂从 pan 轴算
        #      最大臂展只有 REACH_MAX≈0.374 m，z 拉开就够不着、固件只会回 MAX_ITER。
        #      径向夹进球内不改方位角，所以下面的 yaw 约束仍然成立。----
        if c.get('reach', {}).get('enable', True):
            xyz, over_m = clamp_reach(xyz, float(c.get('reach', {}).get('margin_m', 0.030)))
            if over_m > 1e-4:
                flags.append('reach')

        # ---- roll / pitch：源可选（翻腕/转腕退化问题的对策）
        #   roll.source  = 'theta'（画面内旋转，默认） | 'psi'（法线出平面分量≈前臂旋前/旋后）
        #   pitch.source = 'phi'（法线去旋后倾角，默认） | 'v'（改用掌心纵向位置，完全避开旋转歧义）
        roll_src = str(rc.get('source', 'cuff2d'))
        pitch_src = str(pc.get('source', 'hold'))
        if roll_src == 'cuff2d':
            d_roll = math.degrees(wrap(cuff2d - math.radians(z0.get('cuff2d_deg', 0.0))))
        elif roll_src == 'cuff':
            d_roll = math.degrees(wrap(cuff - math.radians(z0.get('cuff_deg', 0.0))))
        elif roll_src == 'palm':
            d_roll = math.degrees(alpha - math.radians(z0.get('palm_deg', 0.0)))
        elif roll_src == 'twist':
            d_roll = math.degrees(wrap(twist - math.radians(z0.get('twist_deg', 0.0))))
        elif roll_src == 'psi':
            d_roll = math.degrees(wrap(psi - math.radians(z0.get('psi_deg', 0.0))))
        else:
            d_roll = math.degrees(wrap(theta - math.radians(z0['theta_deg'])))
        if pitch_src == 'hold':                       # 只留 roll：pitch 固定在起始角不动
            d_pitch = 0.0
        elif pitch_src == 'point':
            d_pitch = math.degrees(wrap(point - math.radians(z0.get('point_deg', 0.0))))
        elif pitch_src == 'v':
            d_pitch = (z0['v'] - sv) * float(pc.get('v_scale_deg', 90.0))
        else:
            d_pitch = math.degrees(wrap(phi - math.radians(z0['phi_deg'])))
        roff = rc['sign'] * rc['gain'] * apply_deadzone(d_roll, rc['deadzone_deg'])
        moff = float(rc.get('max_offset_deg', 90.0))
        if abs(roff) > moff:                        # 到顶就饱和，不再往外跑
            roff = clamp(roff, -moff, moff)
            flags.append('roll_max')
        self.roll_off = roff
        roll = self.start['roll'] + math.radians(roff)
        pitch = self.start['pitch'] + pc['sign'] * pc['gain'] * math.radians(
            apply_deadzone(d_pitch, pc['deadzone_deg']))
        soft = float(pc['soft_limit_rad'])
        if abs(pitch) > soft:
            pitch = clamp(pitch, -soft, soft)
            flags.append('pitch_soft')
        roll = clamp(roll, *FW_LIMIT['roll'])

        # ---- 双限幅：位置/旋转都按 dt 小步爬行（right_joycon.Mapper 同策略）----
        dt = max(1e-3, min(0.5, dt))
        d = [xyz[i] - self.prev['xyz'][i] for i in range(3)]
        length = math.sqrt(sum(t * t for t in d))
        cap_t = c['slew']['max_translation_m_s'] * dt
        if length > cap_t > 0:
            xyz = [self.prev['xyz'][i] + d[i] * cap_t / length for i in range(3)]
            flags.append('slew_xyz')
        cap_r = c['slew']['max_rotation_rad_s'] * dt
        roll = wrap(self.prev['roll'] + clamp(wrap(roll - self.prev['roll']), -cap_r, cap_r))
        pitch = clamp(self.prev['pitch'] + clamp(wrap(pitch - self.prev['pitch']), -cap_r, cap_r),
                      *FW_LIMIT['pitch'])

        # ---- yaw：位置定稿之后再由位置反算（绝不由手给），取靠近当前 yaw 的 pi 分支 ----
        #      MUST 放在 xyz 限速之后：早算就会用未限速的位置，指令 yaw 领先实际位置，
        #      目标离开 5 轴可达流形（yaw=atan2(y,x-0.0612)）→ 固件只会回 MAX_ITER、臂卡住。
        yaw = math.atan2(xyz[1], xyz[0] - SHOULDER_X)
        yaw = min((wrap(yaw), wrap(yaw + math.pi)),
                  key=lambda val: abs(wrap(val - self.prev_yaw)))

        # ---- 夹爪：伸指数 5 -> 张开，0 -> 闭合，其他手势夹爪必须不动 ----
        gc = c['gripper']
        if g['n_ext'] == 5:
            self.gripper = float(gc['open'])
        elif g['n_ext'] == 0:
            self.gripper = float(gc['closed'])
        elif not gc['hold_others']:
            self.gripper = float(gc['init'])
        gripper = clamp(self.gripper, *FW_LIMIT['gripper'])

        out = {'x': xyz[0], 'y': xyz[1], 'z': xyz[2], 'roll': wrap(roll),
               'pitch': clamp(pitch, *FW_LIMIT['pitch']), 'yaw': wrap(yaw), 'gripper': gripper,
               'palm_deg': math.degrees(alpha), 'pal_r': float(g.get('pal_r', 0.0)),
               'point_deg': math.degrees(point), 'cuff_deg': math.degrees(cuff),
               'cuff2d_deg': math.degrees(cuff2d),
               'size_ratio': self.size_ratio, 'd_roll_deg': d_roll,
               'roll_off_deg': self.roll_off}
        for k in ('roll', 'pitch', 'yaw', 'gripper'):           # 兜底：任何越界都记一笔
            if not FW_LIMIT[k][0] <= out[k] <= FW_LIMIT[k][1]:
                flags.append(k + '_fw')
                out[k] = clamp(out[k], *FW_LIMIT[k])
        self.prev = {'xyz': list(xyz), 'roll': out['roll'], 'pitch': out['pitch'], 'yaw': out['yaw']}
        self.prev_yaw = out['yaw']
        self.dbg = {'theta_deg': math.degrees(theta), 'phi_deg': math.degrees(phi),
                    'psi_deg': math.degrees(psi), 'twist_deg': math.degrees(twist),
                    'palm_deg': math.degrees(alpha), 'pal_r': float(g.get('pal_r', 0.0)),
                    'd_roll_deg': d_roll, 'd_pitch_deg': d_pitch, 'u': su, 'v': sv,
                    'roll_src': roll_src, 'pitch_src': pitch_src}
        return out, flags

    def hold(self):
        """丢手时冻结输出（保位置、冻结角度，绝不下发部分解）。"""
        o = self.prev
        return {'x': o['xyz'][0], 'y': o['xyz'][1], 'z': o['xyz'][2], 'roll': o['roll'],
                'pitch': o['pitch'], 'yaw': o['yaw'], 'gripper': self.gripper}


def _avg_geo(frames):
    """多帧几何量取平均（角量走 sin/cos 平均），用于自动标定更稳的零位。"""
    if not frames:
        return None
    if len(frames) == 1:
        return frames[0]
    keys_ang = {'theta', 'phi', 'psi', 'twist', 'point', 'cuff', 'cuff2d'}
    out = {}
    for k in frames[0]:
        vals = [float(f[k]) for f in frames if k in f and np.isscalar(f[k])]
        if not vals:
            out[k] = frames[-1][k]
            continue
        if k in keys_ang:
            out[k] = math.atan2(float(np.mean(np.sin(vals))), float(np.mean(np.cos(vals))))
        else:
            out[k] = float(np.mean(vals))
    out['palm'] = list(np.mean([np.asarray(f['palm'], dtype=float) for f in frames], axis=0))
    return out


def apply_zero(calib, g):
    """把当前/平均后的几何写进 calib['zero']（按键 c 与自动标定共用）。"""
    # 手长基准只能在合理区间（0.03～0.60 归一）才收，否则一次误检就把 x 通道弄废
    hp0 = float(g.get('hand_px', 0.0))
    if not MIN_HAND_PX < hp0 < 0.60:
        old = float((calib.get('zero') or {}).get('hand_px0', 0.0))
        hp0 = old if 0.03 < old < 0.60 else 0.0
    calib['zero'] = {'theta_deg': math.degrees(g['theta']),
                     'phi_deg': math.degrees(g['phi']),
                     'psi_deg': math.degrees(g['psi']),
                     'twist_deg': math.degrees(g['twist']),
                     'palm_deg': float(g.get('palm_deg', 0.0)),   # 已是度
                     'point_deg': math.degrees(g.get('point', 0.0)),
                     'cuff_deg': math.degrees(g.get('cuff', 0.0)),
                     'cuff2d_deg': math.degrees(g.get('cuff2d', 0.0)),
                     'palm_r0': float(g.get('pal_r', 0.86)),
                     'hand_px0': hp0,
                     'u': float(g['palm'][0]), 'v': float(g['palm'][1])}
    return calib['zero']


class AutoZero:
    """握拳 N 次 -> 张开手掌 -> 多帧平均自动采零位（演示前重设相对基准用）。

    历史问题（“看上去失效”的三个真因）：
      1. window_s 1.2s 太紧：两次握拳加上抬手常常来不及，过期就默默清零；
      2. 完全没进度反馈：只成功时才闪一下，第一次握拳、差一次都看不到；
      3. 单帧噪声直接算一次握拳：n_ext 在 0/2 之间抖一下就重复计数或漏计。

    现在：
      - window_s 默认 2.5s（配置可改），每次握拳都闪 '握拳 1/2'；
      - 握拳要连续 hold_frames 帧才算一 次（默认 2），挡单帧噪声；
      - 手丢了 gap_reset_s 以上就重置计数（否则 last_fist 会卡住）。
    """

    def __init__(self, cfg):
        self.n = max(1, int(cfg.get('grasps', 2)))
        self.win = float(cfg.get('window_s', 2.5))
        self.wait_open = float(cfg.get('open_wait_s', 4.0))
        self.avg_n = max(1, int(cfg.get('average_frames', 5)))
        self.cooldown = float(cfg.get('cooldown_s', 2.5))
        self.enabled = bool(cfg.get('enable', True))
        self.hold_n = max(1, int(cfg.get('hold_frames', 2)))
        self.gap_reset = float(cfg.get('gap_reset_s', 0.4))
        self.min_px = float(cfg.get('min_hand_px', MIN_HAND_PX))
        self.times, self.acc = [], []
        self.armed_at = None
        self.cool_until = 0.0
        self.fist_run = 0
        self.last_g_at = 0.0
        self.flash, self.flash_until = '', 0.0

    def _say(self, txt, now, hold=2.2):
        self.flash, self.flash_until = txt, now + hold

    def reset(self):
        self.times, self.acc = [], []
        self.armed_at = None
        self.fist_run = 0

    def feed(self, g, now):
        """返回需要采零位的几何量（已平均），否则 None。"""
        if not self.enabled:
            return None
        if g is None:                                   # 手丢了
            if self.last_g_at and now - self.last_g_at > self.gap_reset:
                if self.fist_run:
                    self.fist_run = 0
                if self.armed_at is not None:
                    self.reset()
                    self._say('手丢了，自动标定取消', now)
            return None
        self.last_g_at = now

        ne = int(g.get('n_ext', 0))
        hpx = float(g.get('hand_px', 0.0))
        if hpx < self.min_px:                 # 手太小/误检：不计数，否则零位基准会被偷走
            self.fist_run = 0
            if self.armed_at is not None:
                self.reset()
            if now >= self.flash_until:       # 不要每帧刷提示（位图缓存按字符串存）
                self._say('手太小（%.0fpx），凑近摄像头再握拳' % (hpx * 640), now, hold=1.5)
            return None
        fist = ne <= 1              # 真实握拳偶尔剩 1 根误判，放宽到 <=1
        opened = ne >= 4
        self.fist_run = self.fist_run + 1 if fist else 0
        new_grasp = (self.fist_run == self.hold_n)         # 每次闭合只计一次

        if new_grasp and self.armed_at is None:
            if now < self.cool_until:
                self._say('刚标过，%.0fs 后再试' % (self.cool_until - now), now, hold=1.4)
            else:
                self.times = [t for t in self.times if now - t <= self.win] + [now]
                if len(self.times) >= self.n:
                    self.times = []
                    self.armed_at, self.acc = now, []
                    self._say('好！张开手掌就标定', now, hold=3.0)
                else:
                    self._say('握拳 %d/%d' % (len(self.times), self.n), now, hold=1.6)
        if self.armed_at is None:
            return None
        if now - self.armed_at > self.wait_open:
            self.reset()
            self._say('自动标定超时（没等到张开手掌）', now)
            return None
        if not opened:
            return None
        self.acc.append(g)
        if len(self.acc) < self.avg_n:
            return None
        nf = len(self.acc)
        avg = _avg_geo(self.acc)
        self.reset()
        self.cool_until = now + self.cooldown
        self._say('已自动标定零位（%d 帧平均）' % nf, now, hold=2.5)
        return avg


def encode_pose1(seq, core, pose):
    """与主机 right_joycon.encode() 同格式：POSE1 <seq> <core> x y z roll pitch yaw gripper"""
    vals = [pose['x'], pose['y'], pose['z'], pose['roll'], pose['pitch'], pose['yaw'],
            pose['gripper']]
    return 'POSE1 %d %d ' % (seq, core) + ' '.join('%.9g' % v for v in vals)


def close_quietly(obj, name='landmarker', timeout=1.5):
    """mediapipe 1.1.1 的 close() 在 VIDEO 模式下会卡 ~40s（实测），放 daemon 线程里限时等。
    不掉也能干净退出：进程结束时资源由 OS 回收（实测 gc 0.02s、退出不等）。"""
    if obj is None:
        return
    import threading
    th = threading.Thread(target=obj.close, daemon=True)
    th.start()
    th.join(timeout)
    if th.is_alive():
        print(f'[提示] {name}.close() 超过 {timeout}s 未返回（mediapipe 已知问题），不等它，直接退出')


def dshow_names():
    """DirectShow 里的摄像头名字，顺序与 cv2 的 index 一致；没有 pygrabber 就返回空表。"""
    try:
        from pygrabber.dshow_graph import FilterGraph
        return list(FilterGraph().get_input_devices())
    except Exception:
        return []


def _looks_builtin(name):
    n = (name or '').lower()
    return any(t in n for t in ('asus', 'ir camera', 'integrated', 'fhd webcam', 'laptop'))


def list_cameras(max_idx=6, backend=None):
    """枚举能真正出帧的摄像头，返回 (序号, 设备名, 宽, 高, 后端)。"""
    if backend is None:
        backend = cv2.CAP_DSHOW if sys.platform.startswith('win') else cv2.CAP_ANY
    names = dshow_names()
    found = []
    for idx in range(max_idx + 1):
        c = cv2.VideoCapture(idx, backend)
        if not c.isOpened():
            c.release()
            continue
        ok, f = c.read()
        if ok and f is not None:
            nm = names[idx] if idx < len(names) else '?'
            found.append((idx, nm, int(f.shape[1]), int(f.shape[0]), c.getBackendName()))
        c.release()
    return found


def pick_auto(cams):
    """auto: 优先选“不像内置”的那只；都像内置就取最后一个。"""
    ext = [c for c in cams if not _looks_builtin(c[1])]
    return (ext[-1] if ext else cams[-1])[0]


class ConsoleKeys:
    """从控制台窗口读键（Windows msvcrt）。
    动因：cv2.waitKey 只能拿到“HighGUI 窗口有焦点时”的键；若焦点在控制台或别的窗口，
    按键根本没到进程，窗口里按什么都没反应。两路一起轮询就没这问题。"""

    def __init__(self):
        self.m = None
        if not sys.platform.startswith('win'):
            return
        try:
            import msvcrt
            if sys.stdin is not None and sys.stdin.isatty():
                self.m = msvcrt
        except Exception:
            self.m = None

    def get(self):
        if self.m is None:
            return -1
        try:
            ch = None
            while self.m.kbhit():
                ch = self.m.getwch()
            if not ch:
                return -1
            if ch in ('\x00', '\xe0'):          # 功能键是两字节，丢掉
                if self.m.kbhit():
                    self.m.getwch()
                return -1
            return ord(ch[0])
        except Exception:
            self.m = None
            return -1


def open_capture(src, width, height, fps):
    if src.isdigit():
        idx = int(src)
        backs = ([('DSHOW', cv2.CAP_DSHOW), ('MSMF', cv2.CAP_MSMF), ('ANY', cv2.CAP_ANY)]
                 if sys.platform.startswith('win') else [('ANY', cv2.CAP_ANY)])
        for name, be in backs:
            c = cv2.VideoCapture(idx, be)
            if c.isOpened():
                ok, _ = c.read()
                if ok:
                    print(f'[采集] 后端 {name}')
                    c.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    c.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    c.set(cv2.CAP_PROP_FPS, fps)
                    c.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    return c
            c.release()
        return None
    return cv2.VideoCapture(src)


def draw(vis, res, g, pose, line, flags, calib, hd, hscore, painter, fps_disp, W, H,
         frozen=False, flash=''):
    """左上中文信息面板：只显示真正参与控制的量。"""
    PAD = 10
    XA, XB = 10, 170             # 两列左端
    WA, WB = 150, 140            # 列宽（数值右对齐到列右缘）
    PW, PH = 330, 224
    LBL = (150, 160, 170)
    VAL = (240, 240, 240)
    HDR = (130, 190, 255)
    DEG = (0, 235, 235)
    TITLE = (255, 255, 255)
    texts, seps = [], []

    def cell(x, w, y, label, value, cl=LBL, cv_=VAL, size=13):
        if label:
            texts.append((label, (x, y), cl, size))
        if value:
            texts.append((value, (x + w - painter.measure(value, size), y), cv_, size))

    def row(y, a, b, size=13):
        cell(XA, WA, y, a[0], a[1], size=size, cv_=a[2] if len(a) > 2 else VAL)
        cell(XB, WB, y, b[0], b[1], size=size, cv_=b[2] if len(b) > 2 else VAL)

    def deg(v):
        return f'{math.degrees(v):+.1f}\u00b0'

    # 手形
    if g is None:
        gtxt, gcol = '未检出（保持不动）', (0, 160, 255)
    elif g['n_ext'] == 5:
        gtxt, gcol = '张开 → 夹爪张开', (80, 220, 80)
    elif g['n_ext'] == 0:
        gtxt, gcol = '握拳 → 夹爪闭合', (90, 90, 240)
    else:
        gtxt, gcol = '其他 → 夹爪保持', (0, 200, 255)
    hdtxt = ('右手' if hd == 'Right' else ('左手' if hd == 'Left' else '未检出'))

    # 面板暗底（先压暗再画骨架，手在面板上层）
    roi = vis[0:PH, 0:PW]
    cv2.addWeighted(roi, 0.32, np.full_like(roi, 14), 0.68, 0, roi)
    if g is not None:
        pts = [(int(l.x * W), int(l.y * H)) for l in res.hand_landmarks[0]]
        for a, b in HAND_LINKS:
            cv2.line(vis, pts[a], pts[b], (0, 200, 255), 2, cv2.LINE_AA)
        for i, (px, py) in enumerate(pts):
            col = (0, 0, 255) if i == 0 else ((80, 220, 80) if i in TIP else (255, 255, 255))
            cv2.circle(vis, (px, py), 4 if i in TIP else 3, col, -1)
        cx, cy = int(g['palm'][0] * W), int(g['palm'][1] * H)
        cv2.drawMarker(vis, (cx, cy), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.ellipse(vis, (cx, cy), (34, 34), -math.degrees(g['theta']), 0, 360, (255, 0, 255), 1)

    # ---- 标题 ----
    ftxt = f'FPS {fps_disp:.1f}'
    texts.append(('手部遥操作 → 机械臂', (PAD, 6), TITLE, 15))
    texts.append((ftxt, (PAD + PW - 12 - painter.measure(ftxt, 12), 8), (190, 245, 190), 12))
    texts.append((f'{hdtxt} · {gtxt}', (PAD, 28), gcol, 14))

    # ---- 手部输入 ----
    seps.append(50)
    texts.append(('手部输入', (PAD, 54), HDR, 11))
    if g is not None:
        row(72, ('横向 u', f'{g["palm"][0] * 100:.1f}%'),
            ('纵向 v', f'{g["palm"][1] * 100:.1f}%'))
        row(90, ('手长比', f"{(pose.get('size_ratio', 1.0) - 1.0) * 100:+.1f}%"),
            ('伸指数', f'{g["n_ext"]}/5'))
        row(108, ('手腕转角', deg(g['theta'])),
            ('相对零位', f"{pose.get('d_roll_deg', 0.0):+.1f}\u00b0"))
    else:
        row(72, ('横向 u', '-'), ('纵向 v', '-'))
        row(90, ('手长比', '-'), ('伸指数', '-'))
        row(108, ('手腕转角', '-'), ('相对零位', '-'))

    # ---- 下发 ----
    seps.append(128)
    texts.append(('下发机械臂（POSE1）', (PAD, 132), HDR, 11))
    row(150, ('左右 y', f'{pose["y"] * 100:.1f}cm'), ('上下 z', f'{pose["z"] * 100:.1f}cm'))
    row(168, ('前后 x', f'{pose["x"] * 100:.1f}cm'),
        ('腕转 roll', deg(pose['roll']), DEG))
    row(186, ('夹爪', f"{pose['gripper']:.1f}",
              (80, 220, 80) if pose['gripper'] > 0.5 else (90, 90, 240)),
        ('俯仰', '固定', (150, 150, 150)))
    _fn = []
    if any(f in ('x', 'y', 'z') for f in flags):
        _fn.append('限速中')
    if any(str(f).endswith('_fw') for f in flags):
        _fn.append('触硬限')
    if 'roll_max' in flags:
        _fn.append('腕转到顶')
    stxt = ('、'.join(_fn) if _fn else ('冻结' if frozen else '正常'))
    cell(XA, WA + WB - XA, 204, '状态', stxt,
         cv_=(0, 150, 255) if (_fn or frozen) else (170, 200, 170), size=12)

    for sy in seps:
        cv2.line(vis, (PAD, sy), (PAD + PW - 14, sy), (85, 92, 102), 1)

    # ---- 底栏 ----
    texts.append(('原始 ' + line, (PAD, H - 62), (180, 180, 180), 11))
    texts.append(('握拳两次自动采零位   c 立即采零位   w 存标定   s 快照   r 录像   空格 冻结   q 退出',
                  (PAD, H - 42), (175, 175, 175), 12))
    texts.append(('方向反了就按：x 腕转   y 左右   v 上下   z 前后   g/G 增益   d/D 死区',
                  (PAD, H - 22), (175, 175, 175), 12))
    vis = painter.flush(vis, texts)

    # 顶部居中提示（自动标定状态）
    if flash:
        fs = 20
        tw = int(painter.measure(flash, fs))
        x0 = max(8, (W - tw) // 2)
        cv2.rectangle(vis, (x0 - 14, 10), (x0 + tw + 14, 10 + fs + 18), (30, 110, 30), -1)
        vis = painter.flush(vis, [(flash, (x0, 16), (235, 255, 235), fs)])

    # 右下：腕转行程条
    bx, bw = W - 240, 160
    y0 = H - 40
    span = float(calib['roll'].get('max_offset_deg', 90.0))
    off = float(pose.get('roll_off_deg', 0.0))
    cv2.rectangle(vis, (bx, y0), (bx + bw, y0 + 14), (70, 70, 70), 1)
    mid = bx + bw // 2
    cv2.line(vis, (mid, y0 - 2), (mid, y0 + 16), (150, 150, 150), 1)
    xr = mid + int(clamp(off / max(1e-6, span), -1, 1) * (bw / 2))
    sat = abs(off) >= span * 0.98
    col = (0, 140, 255) if sat else (80, 220, 80)
    cv2.rectangle(vis, (min(mid, xr), y0 + 2), (max(mid, xr), y0 + 12), col, -1)
    lab = f"腕转偏移 {off:+.0f}°" + (' 到顶' if sat else '')
    vis = painter.flush(vis, [(lab, (bx - 132, y0 + 1), col if sat else (220, 220, 220), 12)])

    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='0',
                    help='摄像头序号(0/1/2..) 或视频路径；写 auto 自动避开内置摄像头')
    ap.add_argument('--list-cams', dest='list_cams', action='store_true',
                    help='只枚举可用摄像头然后退出')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--model', default=str(DEFAULT_MODEL))
    ap.add_argument('--calib', default=str(DEFAULT_CALIB))
    ap.add_argument('--out', default=str(DEFAULT_OUT))
    ap.add_argument('--core', type=int, default=1, choices=[0, 1], help='POSE1 第 2 字段(PL 核选择)')
    ap.add_argument('--num-hands', type=int, default=1)
    ap.add_argument('--hand-swap', dest='hand_swap', action='store_true', default=None,
                    help='强制 handedness 标签互换（MediaPipe 假定输入是镜像自拍，验证后决定）')
    ap.add_argument('--select-hand', default=None, choices=['Left', 'Right'],
                    help='只用这一只手（另一只手忽略）')
    ap.add_argument('--min-det', type=float, default=0.5)
    ap.add_argument('--min-pres', type=float, default=0.5)
    ap.add_argument('--min-track', type=float, default=0.5)
    ap.add_argument('--roll-source', dest='roll_source', default=None,
                    choices=['cuff2d', 'cuff', 'palm', 'twist', 'theta'],
                    help='roll 来源: cuff2d=虎口画面内(默认) / cuff=虎口3D / palm=掌心朝向 / twist=绕臂轴')
    ap.add_argument('--pitch-source', dest='pitch_source', default=None,
                    choices=['hold', 'point', 'phi', 'v'],
                    help='pitch 来源: hold=固定不动(默认,只留roll) / point=爪头仰角 / '
                         'phi=法线倾角 / v=掌心纵向位置')
    ap.add_argument('--no-window', action='store_true', help='不开窗口(自检)')
    ap.add_argument('--no-jsonl', action='store_true', help='不写 jsonl')
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--record', default=None, help='带标注录像 mp4 路径')
    ap.add_argument('--print-every', type=int, default=3, help='每 N 帧打印一次 POSE1(默认 3)')
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    if args.list_cams:
        cams = list_cameras()
        if not cams:
            print('[摄像头] 没找到能出帧的摄像头')
            return 4
        print('[摄像头] 可用序号:')
        for idx, nm, w, h, b in cams:
            tag = '内置' if _looks_builtin(nm) else '外接?'
            print(f'   --source {idx}    {nm:<16} {tag}  {w}x{h}  {b}')
        print('  用法: --source <序号>    或  --source auto（自动避开内置）')
        return 0
    if str(args.source).strip().lower() == 'auto':
        cams = list_cameras()
        if not cams:
            print('[错误] auto 没找到摄像头')
            return 4
        for idx, nm, w, h, b in cams:
            tag = '内置' if _looks_builtin(nm) else '外接?'
            print(f'[摄像头] {idx}: {nm:<16} {tag} {w}x{h} {b}')
        args.source = str(pick_auto(cams))
        sel = [c for c in cams if c[0] == int(args.source)][0]
        print(f'[摄像头] auto -> 选 {args.source}（{sel[1]}）')
        if _looks_builtin(sel[1]):
            print('[警告] 没找到外接摄像头（可能没插好/USB 掉电/被别的程序占用），'
                  '现在用的是内置摄像头')

    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions

    model_path = Path(args.model)
    if not model_path.exists():
        print(f'[错误] 模型不存在: {model_path}')
        print('  下载: curl -L -o model/data/external/mediapipe/hand_landmarker.task '
              'https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task')
        return 2

    out_dir = Path(args.out)
    (out_dir / 'snapshots').mkdir(parents=True, exist_ok=True)
    calib = load_calib(args.calib, quiet=args.quiet)
    az = AutoZero(calib.get('auto_zero', {}))
    if az.enabled:
        print(f'[自动标定] 快速握拳 {az.n} 次后张开手掌 -> 自动采零位（{az.avg_n} 帧平均）')
    if args.hand_swap is not None:
        calib['handedness']['swap'] = bool(args.hand_swap)
    if args.select_hand is not None:
        calib['handedness']['select'] = args.select_hand
    if args.roll_source is not None:
        calib['roll']['source'] = args.roll_source
    if args.pitch_source is not None:
        calib['pitch']['source'] = args.pitch_source
    print(f'[旋转源] roll←{calib["roll"].get("source", "cuff2d")}  '
          f'pitch←{calib["pitch"].get("source", "hold")}')
    print(f'[手性] swap={calib["handedness"]["swap"]} select={calib["handedness"]["select"]}')
    mapper = PoseMapper(calib)
    con_keys = ConsoleKeys()
    print('[按键] 控制台读键 %s（窗口有焦点走 cv2.waitKey，控制台有焦点走 msvcrt）'
          % ('可用' if con_keys.m is not None else '不可用'))

    win = 'B1 pose_map - mediapipe hand -> POSE1'
    if not args.no_window:
        try:
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(win, np.zeros((8, 8, 3), np.uint8))        # 8x8 dummy 试 GUI
            cv2.waitKey(1)
        except cv2.error as e:
            print('[错误] 当前 opencv 没有 GUI 后端(多半装了 headless 版):', str(e)[:200])
            print('       修法: pip uninstall -y opencv-python-headless opencv-contrib-python-headless')
            print('             pip install opencv-python')
            return 3

    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.VIDEO,
        num_hands=args.num_hands,
        min_hand_detection_confidence=args.min_det,
        min_hand_presence_confidence=args.min_pres,
        min_tracking_confidence=args.min_track)
    landmarker = vision.HandLandmarker.create_from_options(opts)
    print(f'[模型] HandLandmarker 已加载 ({model_path.stat().st_size / 1e6:.2f} MB), VIDEO 模式')

    cap = open_capture(args.source, args.width, args.height, args.fps)
    if cap is None or not cap.isOpened():
        print(f'[错误] 打不开视频源: {args.source}')
        return 4
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
    print(f'[采集] {W}x{H} 源={args.source}')

    run_ts = time.strftime('%Y%m%d_%H%M%S')
    jsonl_path = out_dir / f'pose_run_{run_ts}.jsonl'
    jsonl = None if args.no_jsonl else open(jsonl_path, 'w', encoding='utf-8')
    painter = make_painter()
    print('[界面] 文字模式:', '中文(PIL)' if painter.cjk else 'ASCII(cv2)')

    writer = None
    if args.record:
        Path(args.record).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (W, H))

    votes, cmap = deque(maxlen=max(1, calib['smooth']['vote'])), max(1, calib['smooth']['vote'])
    t0 = time.time()
    last_ts_ms, n, seq = -1, 0, 0
    fps_disp, freeze, frozen = 0.0, False, None
    line = 'POSE1 (未开始)'
    last_g = None
    stats = {'frames': 0, 'hand': 0, 'gesture': {}, 'handedness': {}, 'clamped': 0}
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('[结束] 视频源读完')
                break
            if freeze and frozen is not None:
                frame = frozen.copy()
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            now = time.monotonic()
            ts_ms = max(last_ts_ms + 1, int((now - t0) * 1000))   # detect_for_video 要求单调递增
            last_ts_ms = ts_ms
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
            res = landmarker.detect_for_video(mp_img, ts_ms)
            dt = 1.0 / args.fps if mapper.last_t is None else (now - mapper.last_t)
            mapper.last_t = now
            n += 1
            stats['frames'] += 1

            g, hd, hscore = None, None, 0.0
            if res.hand_landmarks:
                k = 0
                if args.num_hands > 1 and res.handedness:
                    k = int(np.argmax([c[0].score for c in res.handedness]))
                if res.handedness:
                    hd = res.handedness[k][0].category_name
                    hscore = float(res.handedness[k][0].score)
                    if calib['handedness']['swap']:
                        hd = 'Left' if hd == 'Right' else 'Right'
                want = calib['handedness']['select']
                if want in ('Left', 'Right') and hd != want:
                    hd = f'{hd}(跳过)'
                else:
                    g = Geometry.compute(res.hand_landmarks[k], res.hand_world_landmarks[k],
                                         calib['extend']['margin_ratio'],
                                         1.0 if hd == 'Right' else -1.0)

            if g is not None:
                stats['hand'] += 1
                if float(calib['zero'].get('hand_px0', 0.0)) <= 1e-6:
                    hp = float(g['hand_px'])
                    if MIN_HAND_PX < hp < 0.60:         # 误检（手很小/在画面边缘）不能当基准
                        calib['zero']['hand_px0'] = hp
                        print(f"[前后] hand_px0 缺失，用当前帧手长 {hp * args.width:.1f}px "
                              "作基准（想换基准请按 c 重采零位）")
                votes.append(g['n_ext'])
                pose, flags = mapper.map(g, dt)
                g['palm_deg'] = pose.get('palm_deg', 0.0)
                last_g = g
                auto_g = az.feed(g, now)
                if auto_g is not None:
                    z = apply_zero(calib, auto_g)
                    mapper.reset_smoothing()
                    stats['auto_zero'] = stats.get('auto_zero', 0) + 1
                    print(f"[自动标定] 零位已更新 u {z['u']:.4f} v {z['v']:.4f} "
                          f"手长 {z['hand_px0'] * args.width:.1f}px "
                          f"theta {z['theta_deg']:+.1f}°（按 w 可存盘）")
            else:
                votes.append(-1)
                pose, flags = mapper.hold(), []

            vals = [v for v in votes if v >= 0]
            voted = max(set(vals), key=vals.count) if len(votes) == cmap and vals else None
            ne = voted if voted is not None else (g['n_ext'] if g else None)
            gesture = '张开 palm' if ne == 5 else ('闭合 fist' if ne == 0 else '其他/保持')

            seq += 1
            line = encode_pose1(seq, args.core, pose)
            stats['gesture'][gesture] = stats['gesture'].get(gesture, 0) + 1
            if hd:
                stats['handedness'][hd] = stats['handedness'].get(hd, 0) + 1
            if flags:
                stats['clamped'] += 1
            if jsonl is not None:
                jsonl.write(json.dumps({
                    'ts_ms': ts_ms, 'seq': seq, 'detected': g is not None,
                    'handedness': hd, 'handedness_score': round(hscore, 4),
                    'theta_deg': round(math.degrees(last_g['theta']), 3) if last_g else None,
                    'phi_deg': round(math.degrees(last_g['phi']), 3) if last_g else None,
                    'psi_deg': round(math.degrees(last_g['psi']), 3) if last_g else None,
                    'twist_deg': round(math.degrees(last_g['twist']), 3) if last_g else None,
                    'pal_r': round(last_g.get('pal_r', 0.0), 4) if last_g else None,
                    'palm_deg': round(pose.get('palm_deg', 0.0), 3),
                    'point_deg': round(pose.get('point_deg', 0.0), 3),
                    'cuff_deg': round(pose.get('cuff_deg', 0.0), 3),
                    'cuff2d_deg': round(pose.get('cuff2d_deg', 0.0), 3),
                    'theta_w_deg': round(math.degrees(last_g['theta_w']), 3) if last_g else None,
                    'normal': [round(x, 4) for x in last_g['normal']] if last_g else None,
                    'palm_uv': [round(last_g['palm'][0], 5), round(last_g['palm'][1], 5)] if last_g else None,
                    'ext': last_g['ext'] if last_g else None,
                    'n_ext': last_g['n_ext'] if last_g else None,
                    'gesture': gesture, 'clamp_flags': flags,
                    'pose1': {k: round(v, 6) for k, v in pose.items()},
                    'pose1_line': line,
                }, ensure_ascii=False) + '\n')
                jsonl.flush()

            vis = draw(frame.copy(), res, g, pose, line, flags, calib, hd, hscore, painter,
                       fps_disp, W, H, frozen=freeze, flash=az.flash if now < az.flash_until else '')
            if writer is not None:
                writer.write(vis)
            if not args.quiet and n % max(1, args.print_every) == 0:
                print(f'[{seq:05d}] {line}' + ('   ' + ','.join(flags) if flags else ''))
            if n % 10 == 0 or n == 1:
                fps_disp = n / max(1e-6, time.time() - t0)

            if not args.no_window:
                cv2.imshow(win, vis)
                k = cv2.waitKey(1) & 0xFF
                ck = con_keys.get()                      # 控制台焦点时的备用读键
                if ck != -1:
                    k = ck
                if k not in (-1, 255):
                    print('[键] %s' % (chr(k) if 32 <= k < 127 else k))
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:   # 用 X 关窗也干净退出
                    print('  窗口已关闭')
                    break
                if k in (ord('q'), 27):
                    break
                elif k == ord(' '):
                    freeze = not freeze
                    frozen = vis.copy() if freeze else None
                elif k == ord('s'):
                    ts = time.strftime('%H%M%S')
                    cv2.imwrite(str(out_dir / 'snapshots' / f'pose_{ts}.png'), vis)
                    print(f'  快照 -> snapshots/pose_{ts}.png')
                elif k == ord('r'):
                    if writer is None:
                        pth = out_dir / f'rec_{time.strftime("%H%M%S")}.mp4'
                        writer = cv2.VideoWriter(str(pth), cv2.VideoWriter_fourcc(*'mp4v'),
                                                 args.fps, (W, H))
                        print(f'  开始录像 -> {pth.name}')
                    else:
                        writer.release()
                        writer = None
                        print('  停止录像')
                elif k == ord('k'):
                    mapper.reset_smoothing()
                    votes.clear()
                    print('  已清空平滑与投票')
                elif k == ord('c') and last_g is not None:
                    apply_zero(calib, last_g)
                    mapper.reset_smoothing()
                    print(f'  采零位 -> theta {calib["zero"]["theta_deg"]:.2f} phi '
                          f'{calib["zero"]["phi_deg"]:.2f} psi {calib["zero"]["psi_deg"]:.2f} '
                          f'指向 {calib["zero"]["point_deg"]:+.1f} '
                          f'虎口 {calib["zero"]["cuff_deg"]:+.1f} '
                          f'u {calib["zero"]["u"]:.4f} v {calib["zero"]["v"]:.4f}')
                elif k == ord('t'):
                    order = ['cuff2d', 'cuff', 'palm', 'twist', 'theta']
                    cur = str(calib['roll'].get('source', 'cuff2d'))
                    calib['roll']['source'] = (order[(order.index(cur) + 1) % len(order)]
                                               if cur in order else 'cuff2d')
                    mapper.reset_smoothing()
                    print(f'  roll 源 -> {calib["roll"]["source"]}'
                          '（cuff2d=虎口画面内；cuff=虎口3D；palm=手心手背翻转；twist=绕臂轴；'
                          'theta=画面内腕转）')
                elif k == ord('u'):
                    order_p = ['hold', 'point', 'v', 'phi']
                    cur = str(calib['pitch'].get('source', 'hold'))
                    calib['pitch']['source'] = (order_p[(order_p.index(cur) + 1) % len(order_p)]
                                                if cur in order_p else 'hold')
                    mapper.reset_smoothing()
                    print(f'  pitch 源 -> {calib["pitch"]["source"]}'
                          '（phi=法线去旋后倾角；v=改用掌心纵向位置，完全避开旋转歧义）')
                elif k == ord('x'):
                    calib['roll']['sign'] *= -1
                    print(f'  roll sign = {calib["roll"]["sign"]:+.0f}')
                elif k == ord('p'):
                    calib['pitch']['sign'] *= -1
                    print(f'  pitch sign = {calib["pitch"]["sign"]:+.0f}')
                elif k == ord('y'):
                    calib['pos']['sign_u'] *= -1
                    print(f'  pos sign_u(左右) = {calib["pos"]["sign_u"]:+.0f}')
                elif k == ord('v'):
                    calib['pos']['sign_v'] *= -1
                    print(f'  pos sign_v(上下) = {calib["pos"]["sign_v"]:+.0f}')
                elif k == ord('z'):
                    calib['pos']['sign_size'] *= -1
                    print(f'  pos sign_size(前后) = {calib["pos"]["sign_size"]:+.0f}')
                elif k == ord('g'):
                    calib['roll']['gain'] = round(max(0.2, calib['roll']['gain'] - 0.1), 2)
                    print(f'  roll gain = {calib["roll"]["gain"]:.2f}')
                elif k == ord('G'):
                    calib['roll']['gain'] = round(calib['roll']['gain'] + 0.1, 2)
                    print(f'  roll gain = {calib["roll"]["gain"]:.2f}')
                elif k == ord('h'):
                    calib['pitch']['gain'] = round(max(0.2, calib['pitch']['gain'] - 0.1), 2)
                    print(f'  pitch gain = {calib["pitch"]["gain"]:.2f}')
                elif k == ord('H'):
                    calib['pitch']['gain'] = round(calib['pitch']['gain'] + 0.1, 2)
                    print(f'  pitch gain = {calib["pitch"]["gain"]:.2f}')
                elif k == ord('d'):
                    calib['roll']['deadzone_deg'] = max(0.0, calib['roll']['deadzone_deg'] - 1)
                    print(f'  roll deadzone = {calib["roll"]["deadzone_deg"]:.0f}')
                elif k == ord('D'):
                    calib['roll']['deadzone_deg'] += 1
                    print(f'  roll deadzone = {calib["roll"]["deadzone_deg"]:.0f}')
                elif k == ord('f'):
                    calib['pitch']['deadzone_deg'] = max(0.0, calib['pitch']['deadzone_deg'] - 1)
                    print(f'  pitch deadzone = {calib["pitch"]["deadzone_deg"]:.0f}')
                elif k == ord('F'):
                    calib['pitch']['deadzone_deg'] += 1
                    print(f'  pitch deadzone = {calib["pitch"]["deadzone_deg"]:.0f}')
                elif k == ord('w'):
                    save_calib(calib, args.calib)
            if args.max_frames and n >= args.max_frames:
                break
    finally:
        el_loop = time.time() - t0
        cap.release()
        if writer is not None:
            writer.release()
        if jsonl is not None:
            jsonl.close()
        close_quietly(landmarker)
        if not args.no_window:
            cv2.destroyAllWindows()

    el = el_loop
    summary = {'frames': stats['frames'], 'seconds': round(el, 2),
               'avg_fps': round(stats['frames'] / el, 1) if el else 0,
               'hand_rate': round(stats['hand'] / max(1, stats['frames']), 3),
               'gesture': stats['gesture'], 'handedness': stats['handedness'],
               'clamped_frames': stats['clamped'], 'source': str(args.source),
               'calib': args.calib, 'jsonl': None if args.no_jsonl else str(jsonl_path),
               'last_line': line}
    (out_dir / f'pose_session_{run_ts}.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
    if not args.quiet:
        print(f'\n[统计] {summary["frames"]} 帧 / {el:.1f}s = {summary["avg_fps"]} fps, '
              f'检出率 {summary["hand_rate"] * 100:.1f}%')
        print(f'  手势分布 {summary["gesture"]}')
        print(f'  手性分布 {summary["handedness"]}')
        print(f'  限幅帧 {summary["clamped_frames"]}')
        print(f'  指令日志 -> {summary["jsonl"]}')
        print(f'  摘要 -> {out_dir}/pose_session_{run_ts}.json')
    return 0


if __name__ == '__main__':
    _code = main()
    # mediapipe 的 C++ 调度线程在解释器退出时会再卡 ~40s（实测 1.1.1）；http 刷完直接硬退
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(_code if isinstance(_code, int) else 0)
