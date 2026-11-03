"""SO-101 5 轴臂 3D 线框仿真：手 -> POSE1 -> IK -> OpenCV 画面里的臂

方案 A（同屏左右分屏）：
  左 = 摄像头 + 手部骨架 + ASCII 关键值（`h` 可切到 pose_map.draw 的完整中文 HUD）
  右 = 自绘 3D 线框臂（球面轨道相机 + 针孔投影，零 3D 依赖）

渲染分三层（为了 30 fps）：
  静态层 bgc : 桌面网格 / 工作空间包络 / 基座 / HUD 面板与中文标签，只在换视角或缩放时重建
  动态层 dyn : 连杆 / 夹爪 / 目标 ghost / 末端轨迹，每帧重画后按掩码贴到 bgc 上
  文字层     : 变化的数值一律 cv2.putText(ASCII)，PIL 中文只进静态层

数据流（与真机主机侧同构，只是不发串口）：
  HandLandmarker -> Geometry.compute -> PoseMapper.map -> POSE1 七元组
                 -> fix_yaw（重算 yaw）-> clamp_reach（夹进可达球）
                 -> kin.solve(热启动, 停滞退出) -> kin.fk_parts -> 画线框

边界（本轮不做）：
  不接 Zynq 板、不下发舵机、不驱动夹爪 ID6、不改 fpga/diag_fw/firmware 任何一行。

按键（窗口有焦点走 cv2.waitKey，控制台有焦点走 msvcrt，双通道）：
  q/ESC 退出   s 快照   r 录像   空格 冻结输入   v 切换视角   -/= 缩放   h 完整HUD
  z 重设前后基准   w 存盘   握拳两次 -> 张开手掌 = 自动标零位
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2] / "src" / "vision"
for _p in (Path(__file__).resolve().parent, ROOT / 'sw'):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import kin                      # noqa: E402  sim/kin.py
import pose_map                 # noqa: E402  sw/pose_map.py（顶层不 import mediapipe，安全）

OUT_DIR = ROOT / 'results' / 'sim'

# ------------------------------------------------------------------ 配色（BGR）
BG = (24, 26, 30)
C_GRID = (44, 48, 56)
C_GRID_MAIN = (68, 76, 92)
C_BOX = (74, 84, 108)
C_LINK = (206, 156, 62)
C_LINK_DIM = (128, 96, 40)
C_JOINT = (232, 232, 236)
C_AXLE = (110, 140, 70)
C_FINGER = (110, 214, 250)
C_EE = (250, 232, 140)
C_TRAIL = (90, 200, 250)
C_TARGET = (210, 120, 255)
C_TARGET_DIM = (170, 96, 200)
C_GOOD = (110, 225, 120)
C_WARN = (60, 170, 255)
C_BAD = (70, 70, 240)
C_TXT = (228, 230, 236)
C_DIM = (146, 152, 162)
C_AX_X = (70, 80, 220)
C_AX_Y = (80, 210, 80)
C_AX_Z = (220, 130, 60)


def unit(v):
    v = np.asarray(v, float)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-12 else np.zeros_like(v)


# ------------------------------------------------------------------ 相机 / 投影
class Cam:
    """球面轨道相机 + 针孔投影，只依赖 numpy/cv2。"""

    PRESETS = (('斜视', 62.0, 20.0), ('侧视', 90.0, 8.0),
               ('正视', 2.0, 8.0), ('俯视', 62.0, 72.0))

    def __init__(self, w, h, fov_deg=38.0, center=(0.18, 0.01, 0.16), dist=0.55):
        self.f = 0.5 * float(h) / math.tan(math.radians(fov_deg) * 0.5)
        self.cx, self.cy = w * 0.5, h * 0.5
        self.center = np.array(center, float)
        self.dist = float(dist)
        self.idx = 0
        self.set_preset(0)

    def set_preset(self, i):
        self.idx = int(i) % len(self.PRESETS)
        self.name, self.yaw, self.pitch = self.PRESETS[self.idx]
        self.update()

    def update(self):
        cy, sy = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        cp, sp = math.cos(math.radians(self.pitch)), math.sin(math.radians(self.pitch))
        self.eye = self.center + self.dist * np.array([cp * cy, cp * sy, sp])
        self.fwd = unit(self.center - self.eye)
        self.right = unit(np.cross(self.fwd, np.array([0.0, 0.0, 1.0])))
        self.up = np.cross(self.right, self.fwd)

    def to_cam(self, p):
        d = np.asarray(p, float) - self.eye
        return np.array([float(d @ self.right), float(d @ self.up), float(d @ self.fwd)])

    def pix(self, c):
        z = max(1e-6, float(c[2]))
        return (self.cx + self.f * float(c[0]) / z, self.cy - self.f * float(c[1]) / z)

    def project_many(self, pts):
        """批量投影，返回 (像素 (N,2) float32, 相机深度 (N,))。"""
        d = np.asarray(pts, float) - self.eye
        z = d @ self.fwd
        zz = np.maximum(z, 1e-6)
        sx = self.cx + self.f * (d @ self.right) / zz
        sy = self.cy - self.f * (d @ self.up) / zz
        return np.stack([sx, sy], axis=1), z


def fade(col, z):
    """按相机深度压暗（近亮远暗），给线框一点纵深。"""
    k = max(0.55, min(1.0, 1.18 - float(z) / 1.55))
    return (int(col[0] * k), int(col[1] * k), int(col[2] * k))


class Scene:
    """三层线框画布：静态层 bgc + 动态层 layer + ASCII 文字表。"""

    def __init__(self, w, h, cam):
        self.w, self.h, self.cam = int(w), int(h), cam
        self.img = np.zeros((self.h, self.w, 3), np.uint8)
        self.img[:] = BG
        self.dyn = np.zeros_like(self.img)
        self.layer = self.dyn            # 当前图元落点（重建静态层时切到 img）
        self.bgc = None
        self.key = None
        self.texts = []                  # 动态 ASCII：(text, xy, col, size)
        self.chtexts = []                # 静态中文（只进 bgc）
        self.hudx = {}

    # ---- 生命周期
    def begin(self):
        self.dyn[:] = 0
        self.texts = []

    def end(self):
        """把动态层合到静态层副本上。

        动态层底色是黑、图元都比底色亮，所以按通道取 max 就等价于「贴上去」，
        比 bool 掩码 + 布尔索引快一个数量级（实测 8.4ms -> 0.4ms）。
        """
        np.maximum(self.bgc, self.dyn, out=self.img)

    def build_static(self, painter, ws_lo, ws_hi):
        """重建静态层：网格 / 包络 / 基座 / HUD 面板 + 中文标签（换视角或缩放才重建）。"""
        key = (self.cam.idx, round(self.cam.dist, 4), self.w, self.h)
        if self.bgc is not None and key == self.key:
            return
        self.img[:] = BG
        self.layer = self.img
        draw_ground(self)
        self.box(ws_lo, ws_hi, C_BOX, 1)
        draw_base(self)
        self.layer = self.dyn
        self.chtexts = []
        build_hud_static(self, painter)
        self.img = painter.flush(self.img, self.chtexts)      # PIL 中文只这一次
        self.bgc = self.img.copy()
        self.key = key

    # ---- 图元（写 self.layer）
    def line(self, a, b, col, thick=2, near=0.035):
        ca, cb = self.cam.to_cam(a), self.cam.to_cam(b)
        if ca[2] < near and cb[2] < near:
            return
        if ca[2] < near:
            t = (near - ca[2]) / (cb[2] - ca[2])
            ca = ca + t * (cb - ca)
        elif cb[2] < near:
            t = (near - cb[2]) / (ca[2] - cb[2])
            cb = cb + t * (ca - cb)
        p1, p2 = self.cam.pix(ca), self.cam.pix(cb)
        c = fade(col, 0.5 * (ca[2] + cb[2]))
        cv2.line(self.layer, (int(round(p1[0])), int(round(p1[1]))),
                 (int(round(p2[0])), int(round(p2[1]))), c, thick, cv2.LINE_AA)

    def dot(self, p, col, r=4, near=0.035):
        c = self.cam.to_cam(p)
        if c[2] < near:
            return
        x, y = self.cam.pix(c)
        cv2.circle(self.layer, (int(round(x)), int(round(y))), r,
                   fade(col, c[2]), -1, cv2.LINE_AA)

    def dashed(self, a, b, col, thick=1, dash=0.030, gap=0.026):
        a, b = np.asarray(a, float), np.asarray(b, float)
        L = float(np.linalg.norm(b - a))
        if L < 1e-9:
            return
        u = (b - a) / L
        t = 0.0
        while t < L - 1e-9:
            self.line(a + u * t, a + u * min(L, t + dash), col, thick)
            t += dash + gap

    def box(self, lo, hi, col, thick=1):
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        E = [((x0, y0, z0), (x1, y0, z0)), ((x1, y0, z0), (x1, y1, z0)),
             ((x1, y1, z0), (x0, y1, z0)), ((x0, y1, z0), (x0, y0, z0)),
             ((x0, y0, z1), (x1, y0, z1)), ((x1, y0, z1), (x1, y1, z1)),
             ((x1, y1, z1), (x0, y1, z1)), ((x0, y1, z1), (x0, y0, z1)),
             ((x0, y0, z0), (x0, y0, z1)), ((x1, y0, z0), (x1, y0, z1)),
             ((x1, y1, z0), (x1, y1, z1)), ((x0, y1, z0), (x0, y1, z1))]
        for a, b in E:
            self.dashed(a, b, col, thick)

    def frame(self, T, size=0.035, thick=2, dim=False):
        p, R = T[:3, 3], T[:3, :3]
        cols = ((C_AX_X, C_AX_Y, C_AX_Z) if not dim
                else (C_TARGET_DIM, C_TARGET_DIM, C_TARGET_DIM))
        for k in range(3):
            self.line(p, p + R[:, k] * size, cols[k], thick)

    def text(self, t, xy, col=C_TXT, size=13):
        self.texts.append((str(t), (int(xy[0]), int(xy[1])), col, int(size)))

    def panel(self, p1, p2, alpha=0.62, col=(12, 14, 18)):
        x0, y0 = max(0, p1[0]), max(0, p1[1])
        x1, y1 = min(self.w, p2[0]), min(self.h, p2[1])
        if x1 <= x0 or y1 <= y0:
            return
        roi = self.img[y0:y1, x0:x1]
        roi[:] = (roi * alpha + np.array(col, float) * (1.0 - alpha)).astype(np.uint8)

    def flush(self, ascii_only=True):
        """把动态 ASCII 文字刷到 img（cv2.putText，比 PIL 快一个数量级）。"""
        for t, xy, col, size in self.texts:
            sc = max(0.32, size / 30.0)
            cv2.putText(self.img, t, (xy[0], xy[1] + int(size * 0.82)),
                        cv2.FONT_HERSHEY_SIMPLEX, sc, col, 1, cv2.LINE_AA)
        return self.img


# ------------------------------------------------------------------ 3D 场景图元
def draw_ground(scene, x0=-0.06, x1=0.50, y0=-0.34, y1=0.34, step=0.05):
    for x in np.arange(x0, x1 + 1e-9, step):
        scene.line((x, y0, 0.0), (x, y1, 0.0), C_GRID, 1)
    for y in np.arange(y0, y1 + 1e-9, step):
        scene.line((x0, y, 0.0), (x1, y, 0.0), C_GRID, 1)
    scene.line((0.0, 0.0, 0.0), (0.10, 0.0, 0.0), C_GRID_MAIN, 1)


def draw_base(scene, col=C_LINK_DIM):
    """基座：底面方板 + 立柱（立柱顶点 = 固件的 shoulder_pan 轴原点）。"""
    s = 0.055
    scene.box((-s, -s, 0.0), (s, s, 0.016), col, 1)
    pan_origin = np.array([kin.TX[0], 0.0, kin.TZ[0]])
    scene.line((0.0, 0.0, 0.016), pan_origin, col, 3)
    scene.dot(pan_origin, C_JOINT, 4)


def draw_gripper(scene, T, gripper, col=C_FINGER, thick=3):
    """按末端系画夹爪：tool = x（腕 roll 轴 / 接近方向），两指沿 y 分开。"""
    p, R = T[:3, 3], T[:3, :3]
    tool, jaw = R[:, 0], R[:, 1]
    root = p + tool * 0.012
    open_w = 0.006 + 0.020 * float(max(0.0, min(1.0, gripper)))
    scene.line(p, root, col, thick)
    scene.line(root - jaw * (open_w * 0.5), root + jaw * (open_w * 0.5), col, 2)
    for s in (-1.0, 1.0):
        a = root + jaw * (s * open_w * 0.5)
        b = a + tool * 0.030 - jaw * (s * open_w * 0.18)
        scene.line(a, b, col, thick)
        scene.line(b, b + tool * 0.008, col, 2)      # 指尖
    scene.dot(p + tool * 0.050, col, 3)


def draw_arm(scene, q, gripper, thick=4):
    origins, axes, T = kin.fk_parts(q)
    pts = [np.zeros(3)] + [np.asarray(o) for o in origins]
    for a, b in zip(pts[:-1], pts[1:]):
        scene.line(a, b, C_LINK, thick)
    for i, (p, ax) in enumerate(zip(origins, axes)):
        scene.line(p - ax * 0.022, p + ax * 0.022, C_AXLE, 1)
        scene.dot(p, C_JOINT, 4 if i else 3)
    draw_gripper(scene, T, gripper)
    return T


def draw_trail(scene, trail, bands=4):
    """轨迹拖尾：分 4 段批量画折线（逐段 cv2.line 太贵）。"""
    n = len(trail)
    if n < 2:
        return
    arr = np.asarray(trail, float)
    px, z = scene.cam.project_many(arr)
    ok = z > 0.05
    for b in range(bands):
        i0 = n * b // bands
        i1 = min(n, n * (b + 1) // bands + 1)
        sel = np.zeros(n, bool)
        sel[i0:i1] = True
        sel &= ok
        if int(sel.sum()) < 2:
            continue
        pts = px[sel].astype(np.int32)
        t = (b + 1) / bands
        col = (int(30 + 60 * t), int(90 + 120 * t), int(140 + 110 * t))
        cv2.polylines(scene.layer, [pts], False, col, 2, cv2.LINE_AA)


def draw_ee_marker(scene, T, col=C_EE):
    p = T[:3, 3]
    scene.dot(p, col, 5)
    scene.dashed(p, (p[0], p[1], 0.0), (70, 90, 110), 1, dash=0.010, gap=0.012)
    scene.dot((p[0], p[1], 0.0), (70, 90, 110), 3)


def draw_target_ghost(scene, T, gripper):
    """目标位姿 ghost：直接用暗一档的颜色画（不再开透明层，省 10ms/帧）。"""
    scene.frame(T, 0.05, thick=2, dim=True)
    draw_gripper(scene, T, gripper, col=C_TARGET_DIM, thick=2)


# ------------------------------------------------------------------ HUD（3D 面板）
def _lbl(scene, painter, t, x, y, col=C_DIM, size=12):
    scene.chtexts.append((t, (x, y), col, size))
    return int(x + painter.measure(t, size) + 6)


def build_hud_static(scene, painter):
    """HUD 的中文标签与面板只画一次，进静态层；变的数值走 ASCII。"""
    W, H = scene.w, scene.h
    PAD = 10
    scene.panel((0, 0), (W, 98), 0.55)
    scene.panel((0, H - 78), (W, H), 0.55)
    x = {}
    scene.chtexts.append(('SO-101 线框仿真（不接板 · 不下发舵机）', (PAD, 5), (255, 255, 255), 15))

    y1 = 30
    xx = PAD
    xx = _lbl(scene, painter, 'FPS', xx, y1)
    x['fps'] = xx
    xx = _lbl(scene, painter, 'IK', xx + 52, y1)
    x['ik'] = xx
    xx = _lbl(scene, painter, '帧', xx + 78, y1)
    x['n'] = xx
    x['view'] = PAD + 330

    y2 = 52
    xx = PAD
    xx = _lbl(scene, painter, '状态', xx, y2)
    x['state'] = xx
    xx = _lbl(scene, painter, '迭代', xx + 96, y2)
    x['it'] = xx
    xx = _lbl(scene, painter, '位置残差mm', xx + 34, y2)
    x['perr'] = xx
    xx = _lbl(scene, painter, '姿态残差deg', xx + 74, y2)
    x['rerr'] = xx

    y3 = 74
    xx = _lbl(scene, painter, '目标', PAD, y3, C_TARGET)
    x['tgt'] = xx

    ya = H - 64
    xx = _lbl(scene, painter, '实际', PAD, ya, C_TXT)
    x['ee'] = xx
    xx = _lbl(scene, painter, '夹爪', PAD + 300, ya, C_DIM)
    x['grip'] = xx
    x['gripbar'] = (xx + 34, ya - 1)

    scene.chtexts.append(('绿=收敛  橙=容差内已接受  红=未收敛沿用   ·   握拳2次→张开手掌=自动标零',
                          (PAD, H - 44), C_DIM, 11))
    scene.chtexts.append(('空格冻结  v视角  -/=缩放  s快照  r录像  h完整HUD  z前后基准  w存盘  q退出',
                          (PAD, H - 23), C_DIM, 11))
    scene.hudx = x


def hud_3d_dynamic(scene, st):
    """每帧变化的数值/状态（全 ASCII，cv2.putText 快）。"""
    x = scene.hudx
    if not x:
        return
    scene.text('%.1f' % st['fps'], (x['fps'], 30), (190, 245, 190), 12)
    scene.text('%.2f ms' % st['ik_ms'], (x['ik'], 30), (190, 245, 190), 12)
    scene.text('%d' % st['n'], (x['n'], 30), C_TXT, 12)
    scene.text('%s %s' % (st['view'], 'FREEZE' if st['freeze'] else ''),
               (x['view'], 30), C_WARN if st['freeze'] else C_DIM, 12)

    code, hold, conv = st['code'], st['hold'], st['conv']
    if code == 0:
        state, col = 'code 0 CONVERGED', C_GOOD
    elif hold:
        state, col = 'code %d HOLD' % code, C_BAD
    else:
        state, col = 'code %d ACCEPT' % code, C_WARN
    scene.text(state, (x['state'], 52), col, 12)
    scene.text('%d' % st['it'], (x['it'], 52), C_TXT, 12)
    scene.text('%.3f' % (st['pos_err'] * 1000.0), (x['perr'], 52), C_TXT, 12)
    scene.text('%.3f' % math.degrees(st['rot_err']), (x['rerr'], 52), C_TXT, 12)

    t = st['target']
    if t is not None:
        scene.text('x%+.3f y%+.3f z%+.3f  roll%+.1f pitch%+.1f yaw%+.1f' %
                   (t[0], t[1], t[2], math.degrees(t[3]), math.degrees(t[4]),
                    math.degrees(t[5])), (x['tgt'], 74), C_TARGET, 12)

    e = st['ee']
    scene.text('x%+.3f y%+.3f z%+.3f' % (e[0], e[1], e[2]), (x['ee'], scene.h - 64),
               C_TXT, 12)
    bx, by = x['gripbar']
    bw = 92
    cv2.rectangle(scene.img, (bx, by + 2), (bx + bw, by + 12), (58, 62, 70), -1)
    fill = int(round(bw * max(0.0, min(1.0, st['gripper']))))
    if fill > 0:
        cv2.rectangle(scene.img, (bx, by + 2), (bx + fill, by + 12), C_FINGER, -1)
    scene.text('%.2f' % st['gripper'], (bx + bw + 8, by - 2), C_FINGER, 12)

    note = st.get('note') or ''
    if st['lim']:
        note = ('越界:' + ','.join(st['lim']) + '  ' + note).strip()
    if note:
        scene.text(note[:80], (10, scene.h - 22), C_WARN, 11)

    if st['hold']:                       # 冻结时打横幅，别让人以为画面坏了
        msg = 'IK HOLD: code %d（沿用上一帧解）' % code
        tw = max(160, int(len(msg) * 9))
        x0 = max(6, (scene.w - tw) // 2)
        cv2.rectangle(scene.img, (x0, 104), (x0 + tw, 136), (24, 70, 128), -1)
        scene.text(msg, (x0 + 12, 108), (60, 235, 255), 15)


# ------------------------------------------------------------------ 左侧面板
class CnChip:
    """中文提示位图缓存：同一句话只让 PIL 渲染一次，之后每帧只做一次拷贝。"""

    def __init__(self, painter):
        self.painter = painter
        self.cache = {}

    def get(self, text, size=16, fg=(60, 235, 255), bg=(18, 20, 26)):
        if not text:
            return None
        key = (text, size, fg)
        if key in self.cache:
            return self.cache[key]
        pad = 7
        w = int(self.painter.measure(text, size)) + 2 * pad
        h = int(size * 1.6) + 2 * pad
        chip = np.zeros((h, max(1, w), 3), np.uint8)
        chip[:] = bg
        if self.painter.cjk:
            chip = self.painter.flush(chip, [(text, (pad, pad), fg, size)])
        else:
            cv2.putText(chip, text.encode('ascii', 'ignore').decode() or '?', (pad, pad + size),
                        cv2.FONT_HERSHEY_SIMPLEX, max(0.34, size / 28.0), fg, 1, cv2.LINE_AA)
        if len(self.cache) > 64:
            self.cache.clear()
        self.cache[key] = chip
        return chip


def blit(left, chip, x=10):
    """把提示位图贴在左面板左下角。"""
    if chip is None or left is None:
        return left
    h, w = chip.shape[:2]
    if h >= left.shape[0] or w >= left.shape[1]:
        return left
    y0 = left.shape[0] - h - 8
    left[y0:y0 + h, x:x + w] = chip
    return left


def panel_camera_fast(frame, res, g, pose, flags, st, hd):
    """快速左面板：摄像头原图 + 手骨架 + ASCII 关键值（约 2ms，替代 22ms 的 PIL HUD）。"""
    img = frame
    H, W = img.shape[:2]
    if g is not None and res is not None and res.hand_landmarks:
        pts = [(int(l.x * W), int(l.y * H)) for l in res.hand_landmarks[0]]
        for a, b in pose_map.HAND_LINKS:
            cv2.line(img, pts[a], pts[b], (0, 200, 255), 2, cv2.LINE_AA)
        for i, (px, py) in enumerate(pts):
            col = (0, 0, 255) if i == 0 else ((80, 220, 80) if i in pose_map.TIP
                                              else (255, 255, 255))
            cv2.circle(img, (px, py), 4 if i in pose_map.TIP else 3, col, -1)
    band = 74
    roi = img[0:band, 0:W]
    roi[:] = (roi * 0.25).astype(np.uint8)
    lines = [
        ('POSE1 x%+.3f y%+.3f z%+.3f roll%+.3f pitch%+.3f yaw%+.3f grip%.2f' %
         (pose['x'], pose['y'], pose['z'], pose['roll'], pose['pitch'], pose['yaw'],
          pose['gripper']), C_TXT),
        ('hand %s  u %5.1f%%  v %5.1f%%  ext %s  theta %s' %
         (hd or '-', g['palm'][0] * 100 if g else -1, g['palm'][1] * 100 if g else -1,
          ('%d/5' % g['n_ext']) if g else '-',
          ('%+.1f' % math.degrees(g['theta'])) if g else '-'), C_DIM),
        ('flags %s%s%s' % (','.join(flags) if flags else 'none',
                           ('   px %.0f' % (float(g['hand_px']) * W)) if g else '',
                           '   FREEZE' if st['freeze'] else ''), C_WARN),
    ]
    for i, (t, c) in enumerate(lines):
        cv2.putText(img, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 2,
                    cv2.LINE_AA)
        cv2.putText(img, t, (8, 20 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.42, c, 1, cv2.LINE_AA)
    return img


def panel_offline(w, h, st, pose, line, flags):
    """离线回放的左面板（无摄像头），全 ASCII，成本约 1ms。"""
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (16, 18, 22)
    for y in range(0, h, 40):
        cv2.line(img, (0, y), (w, y), (24, 26, 30), 1)
    lines = [
        ('OFFLINE REPLAY  (POSE1 -> IK -> render, no camera)', (255, 255, 255)),
        ('file  %s' % st['src'], C_TXT),
        ('frame %d / %d    record ts %d ms%s' % (st['n'], st['total'], st['ts'],
                                                 '   FREEZE' if st['freeze'] else ''), C_TXT),
        ('POSE1 x%+.4f y%+.4f z%+.4f' % (pose['x'], pose['y'], pose['z']), C_TARGET),
        ('      roll%+.4f pitch%+.4f yaw%+.4f grip%.3f' %
         (pose['roll'], pose['pitch'], pose['yaw'], pose['gripper']), C_TARGET),
        ('clamp_flags  %s' % (','.join(flags) if flags else 'none'), C_WARN),
        ('raw  %s' % (line[:92] if line else '-'), C_DIM),
        ('', C_DIM),
        ('space freeze   v view   -/= zoom   s snap   r rec   q quit', C_DIM),
    ]
    for i, (t, c) in enumerate(lines):
        if not t:
            continue
        cv2.putText(img, t, (12, 30 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.44, c, 1,
                    cv2.LINE_AA)
    return img


# ------------------------------------------------------------------ POSE1 -> IK
PAN_ORIGIN = np.array([kin.TX[0], 0.0, kin.TZ[0]], float)      # joint0 轴（shoulder_pan）
# 从 pan 轴出发的最大臂展 = 各连杆长度之和（i>=1，每段长度 hypot(TX,TZ) 与 q 无关）
REACH_FROM_PAN = float(sum(math.hypot(kin.TX[i], kin.TZ[i]) for i in range(1, 5)))


def clamp_reach(pose, on=True, margin=0.030):
    """把目标位置径向夹进以 pan 轴为球心的可达球。

    上游 `pose_map` 的工作空间盒（x 0.12..0.45、z 0.03..0.45）比真实可达域大很多：
    5 轴臂从 pan 轴算起最大臂展只有 REACH_FROM_PAN（≈0.374 m），z 拉到 0.4 就够不着，
    固件/仿真都只会回 code=1 MAX_ITER、臂卡在原地。径向缩放不改方位角，所以
    yaw = atan2(y, x-0.0612) 这条流形约束仍然成立。返回 (夹紧后 pose, 超出量 m)。
    """
    if not on:
        return dict(pose), 0.0
    p = np.array([pose['x'], pose['y'], pose['z']], float)
    d = p - PAN_ORIGIN
    r = float(np.linalg.norm(d))
    lim = REACH_FROM_PAN - float(margin)
    if r <= lim or r < 1e-9:
        return dict(pose), 0.0
    p2 = PAN_ORIGIN + d * (lim / r)
    out = dict(pose)
    out.update(x=float(p2[0]), y=float(p2[1]), z=float(p2[2]))
    return out, r - lim


def fix_yaw(pose):
    """把 yaw 重算成「位置的方位角」，保证 IK 目标落在可达流形上。

    5 轴臂必须满足 yaw = atan2(y, x-0.0612)，否则就是 MAX_ITER。`pose_map.py` 里这个
    yaw 是在**位置 slew 限幅之前**算的，限速一介入指令 yaw 就领先实际位置
    （实测最大偏 47°），目标离开流形 → 臂卡住不动。仿真这边按文档口径重算；
    分支取靠近原 yaw 的那个，避免突然翻 180°。
    """
    out = dict(pose)
    want = math.atan2(float(pose['y']), float(pose['x']) - kin.TX[0])
    cur = float(pose['yaw'])
    d0 = (want - cur + math.pi) % (2.0 * math.pi) - math.pi
    d1 = (want + math.pi - cur + math.pi) % (2.0 * math.pi) - math.pi
    out['yaw'] = kin.wrap_pi(want if abs(d0) <= abs(d1) else want + math.pi)
    return out


def ik_step(q_prev, pose, max_steps, stall=8, accept_pos=0.003, accept_rot=0.005):
    """目标位姿 -> 一次 DLS 迭代求解。

    返回 (q, info, ms, Tt, accepted, converged)：
      converged = 固件同义的 code==0；
      accepted  = converged 或「残差已在容差内」——后者用最后一步迭代值，不冻结；
      两者都否 → 沿用上一帧解（与主机侧同策略）。
    """
    Tt = kin.make_target(pose['x'], pose['y'], pose['z'],
                         pose['roll'], pose['pitch'], pose['yaw'])
    t0 = time.perf_counter()
    r = kin.solve(Tt, q_prev, max_steps=max_steps, tolerance=1e-8, damping=0.01,
                  stall=stall)
    ms = (time.perf_counter() - t0) * 1000.0
    conv = r['code'] == 0
    ok = conv or (r['pos'] <= accept_pos and r['rot'] <= accept_rot)
    return (np.array(r['q'], float) if ok else np.array(q_prev, float)), r, ms, Tt, ok, conv


def replay_rows(path):
    """逐行流式读 jsonl（大文件不整体载入）。"""
    with open(path, 'r', encoding='utf-8') as f:
        for ln in f:
            ln = ln.strip()
            if not ln:
                continue
            try:
                yield json.loads(ln)
            except Exception:
                continue


def row_to_pose(row):
    p = row.get('pose1') or {}
    keys = ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'gripper')
    if all(k in p for k in keys):
        return {k: float(p[k]) for k in keys}
    ln = row.get('pose1_line') or ''
    parts = ln.split()
    if len(parts) >= 10:                      # POSE1 <seq> <core> + 7 个数
        return dict(zip(keys, [float(v) for v in parts[3:10]]))
    return None


# ------------------------------------------------------------------ 主流程
def build_args():
    ap = argparse.ArgumentParser()
    ap.add_argument('--replay', default=None, help='离线回放 jsonl（不起摄像头/不开 landmarker）')
    ap.add_argument('--replay-speed', type=float, default=1.0, help='回放倍速（默认 1.0=按记录时间）')
    ap.add_argument('--replay-loop', action='store_true', help='回放循环播放')
    ap.add_argument('--source', default='auto', help='摄像头序号 或 auto（自动避开内置）')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--model', default=str(pose_map.DEFAULT_MODEL))
    ap.add_argument('--calib', default=str(pose_map.DEFAULT_CALIB))
    ap.add_argument('--num-hands', type=int, default=1)
    ap.add_argument('--panel-w', dest='panel_w', type=int, default=640)
    ap.add_argument('--panel-h', dest='panel_h', type=int, default=480)
    ap.add_argument('--fov', type=float, default=38.0)
    ap.add_argument('--dist', type=float, default=0.55, help='相机到注视点距离(m)，运行中可用 -/= 缩放')
    ap.add_argument('--view', default='斜视', choices=[p[0] for p in Cam.PRESETS],
                    help='初始视角')
    ap.add_argument('--full-hud', dest='full_hud', action='store_true',
                    help='一开始就用 pose_map.draw 的完整中文 HUD（慢约 20ms/帧）')
    ap.add_argument('--max-steps', dest='max_steps', type=int, default=100)
    ap.add_argument('--stall', type=int, default=8,
                    help='能量连续多少步不再下降就判不可达（0=关，跑满 max_steps）')
    ap.add_argument('--no-clamp-reach', dest='clamp_reach', action='store_false',
                    help='关掉「目标夹到可达球」（关掉后超可达目标会停在原地并标 HOLD）')
    ap.add_argument('--reach-margin', dest='reach_margin', type=float, default=0.030,
                    help='可达球内缩量 m（默认 0.030，远离边界才不会逐帧磨 100 次迭代）')
    ap.add_argument('--x-range', dest='x_range', type=float, default=0.08,
                    help='前后(x)相对起始点的最大偏移 m，挡手长比噪声（默认 0.08）')
    ap.add_argument('--accept-pos', dest='accept_pos', type=float, default=0.003,
                    help='未完全收敛但位置残差小于该值(m)就接受该解，不冻结（默认 0.003）')
    ap.add_argument('--accept-rot', dest='accept_rot', type=float, default=0.005,
                    help='同上，姿态残差阈值 rad（默认 0.005 ≈ 0.29°）')
    ap.add_argument('--max-frames', dest='max_frames', type=int, default=0)
    ap.add_argument('--no-window', dest='no_window', action='store_true')
    ap.add_argument('--dump-frame', dest='dump_frame', default=None,
                    help='把最后一帧合成图写到该路径（配合 --no-window 自检）')
    ap.add_argument('--print-every', dest='print_every', type=int, default=30)
    ap.add_argument('--quiet', action='store_true')
    return ap.parse_args()


def main():
    args = build_args()
    live = args.replay is None
    calib = pose_map.load_calib(args.calib, quiet=args.quiet)
    mapper = pose_map.PoseMapper(calib)
    az = pose_map.AutoZero(calib.get('auto_zero', {}))
    painter = pose_map.make_painter()
    chips = CnChip(painter)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / 'snapshots').mkdir(parents=True, exist_ok=True)
    print('[界面] 中文标签用 PIL（只画一次）；动态数值用 cv2.putText')

    cam = Cam(args.panel_w, args.panel_h, fov_deg=args.fov, dist=args.dist)
    cam.set_preset([p[0] for p in Cam.PRESETS].index(args.view))
    scene = Scene(args.panel_w, args.panel_h, cam)
    ws_lo = np.array(pose_map.WORKSPACE_MIN, float)
    ws_hi = np.array(pose_map.WORKSPACE_MAX, float)

    st = dict(mode='实时' if live else '回放', view=cam.name, fps=0.0, ik_ms=0.0, n=0,
              code=0, it=0, pos_err=0.0, rot_err=0.0, hold=False, conv=True, freeze=False,
              target=None, ee=(0.0, 0.0, 0.0), gripper=0.5, lim=[], note='',
              src='', total=0, ts=0)

    win = 'SO-101 sim (live_sim)'
    if not args.no_window:
        try:
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(win, np.zeros((8, 8, 3), np.uint8))
            cv2.waitKey(1)
        except cv2.error as e:
            print('[错误] 当前 opencv 没有 GUI 后端:', str(e)[:200])
            return 3
    con_keys = pose_map.ConsoleKeys()

    cap = landmarker = None
    total = 0
    if live:
        import mediapipe as mp
        from mediapipe.tasks.python import vision, BaseOptions

        if str(args.source).strip().lower() == 'auto':
            cams = pose_map.list_cameras()
            if not cams:
                print('[错误] 没找到能出帧的摄像头')
                return 4
            args.source = str(pose_map.pick_auto(cams))
            sel = [c for c in cams if c[0] == int(args.source)][0]
            print(f'[摄像头] auto -> {args.source}（{sel[1]}）')
        model_path = Path(args.model)
        if not model_path.exists():
            print('[错误] 模型不存在:', model_path)
            return 2
        cap = pose_map.open_capture(args.source, args.width, args.height, args.fps)
        if cap is None or not cap.isOpened():
            print('[错误] 打不开视频源:', args.source)
            return 4
        opts = vision.HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO, num_hands=args.num_hands,
            min_hand_detection_confidence=0.5, min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5)
        landmarker = vision.HandLandmarker.create_from_options(opts)
        print(f'[模型] HandLandmarker 已加载 ({model_path.stat().st_size / 1e6:.2f} MB)')
    else:
        rp = Path(args.replay)
        if not rp.exists():
            print('[错误] 回放文件不存在:', rp)
            return 2
        total = sum(1 for ln in rp.read_text(encoding='utf-8', errors='ignore').splitlines()
                    if ln.strip())
        st['src'] = rp.name
        st['total'] = total
        print(f'[回放] {rp}  {total} 帧  倍速 {args.replay_speed:g}')

    print('[可达性] 可达球半径 %.3f m（从 pan 轴算）；目标夹到边界: %s  内缩 %.0f mm' % (
        REACH_FROM_PAN, '开' if args.clamp_reach else '关', args.reach_margin * 1000))
    print('[流形] yaw 按位置反算重算: 开（pose_map 的 yaw 在 slew 限幅前算的，限速时会偏）')

    q = np.array(kin.HOME_Q, float)
    trail = []
    writer = None
    freeze = False
    frozen_pose = None
    full_hud = bool(args.full_hud)
    t_loop0 = time.time()
    last_ts = -1
    frames = 0
    rp_t0 = rp_first_ms = None
    dump = None
    seq = 0
    stats = dict(frames=0, hand=0, ik_ok=0, ik_acc=0, ik_hold=0, ik_ms=0.0, hand_px=0.0)
    prof = dict(read=0.0, detect=0.0, hud=0.0, ik=0.0, render=0.0, show=0.0)

    def key_handle(k):
        nonlocal writer, freeze, frozen_pose, full_hud
        if k in (ord('q'), 27):
            return False
        if k == ord(' '):
            freeze = not freeze
            print('  冻结输入' if freeze else '  恢复输入')
        elif k == ord('v'):
            cam.set_preset(cam.idx + 1)
            st['view'] = cam.name
            print('  视角 ->', cam.name)
        elif k in (ord('-'), ord('_')):
            cam.dist = min(2.0, cam.dist * 1.15)
            cam.update()
            print('  相机距离 -> %.2f m' % cam.dist)
        elif k in (ord('='), ord('+')):
            cam.dist = max(0.22, cam.dist / 1.15)
            cam.update()
            print('  相机距离 -> %.2f m' % cam.dist)
        elif k == ord('h'):
            full_hud = not full_hud
            print('  左面板 ->', '完整中文 HUD（慢）' if full_hud else '快速 ASCII（快）')
        elif k == ord('k'):
            mapper.reset_smoothing()
            trail.clear()
            print('  已清空平滑与轨迹')
        elif k == ord('w'):
            pose_map.save_calib(calib, args.calib)
        elif k == ord('z'):
            hp = st.get('last_hand_px', 0.0)
            if pose_map.MIN_HAND_PX < hp < 0.60:
                calib['zero']['hand_px0'] = float(hp)
                print('  前后基准 hand_px0 = %.1f px（拿手在舒服的位置重采）' % (hp * 640))
            else:
                print('  当前手长 %.1f px 不可用，先把手放到画面中间再按 z' % (hp * 640))
        elif k == ord('s'):
            pth = OUT_DIR / 'snapshots' / ('sim_%s.png' % time.strftime('%Y%m%d_%H%M%S'))
            if dump is not None:
                cv2.imwrite(str(pth), dump)
                print('  快照 ->', pth.relative_to(ROOT))
        elif k == ord('r'):
            if writer is None and dump is not None:
                pth = OUT_DIR / ('rec_sim_%s.mp4' % time.strftime('%Y%m%d_%H%M%S'))
                writer = cv2.VideoWriter(str(pth), cv2.VideoWriter_fourcc(*'mp4v'),
                                         args.fps, (dump.shape[1], dump.shape[0]))
                print('  开始录像 ->', pth.relative_to(ROOT))
            elif writer is not None:
                writer.release()
                writer = None
                print('  停止录像')
        return True

    row_iter = replay_rows(args.replay) if not live else None
    try:
        while True:
            # ================= 一帧输入 =================
            pose = None
            line, flags, g, hd, res, hscore = '', [], None, None, None, 0.0
            ts_ms = 0
            left = None
            if live:
                _t0 = time.perf_counter()
                ok, frame = cap.read()
                if not ok:
                    print('[结束] 视频源结束')
                    break
                _t1 = time.perf_counter()
                prof['read'] += _t1 - _t0
                now = time.monotonic()
                ts_ms = max(last_ts + 1, int((now - t_loop0) * 1000))
                last_ts = ts_ms
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                mp_img = mp.Image(image_format=mp.ImageFormat.SRGB,
                                  data=np.ascontiguousarray(rgb))
                res = landmarker.detect_for_video(mp_img, ts_ms)
                _t2 = time.perf_counter()
                prof['detect'] += _t2 - _t1
                dt = 1.0 / args.fps if mapper.last_t is None else (now - mapper.last_t)
                mapper.last_t = now
                if res.hand_landmarks:
                    k = 0
                    if res.handedness:
                        if args.num_hands > 1:
                            k = int(np.argmax([c[0].score for c in res.handedness]))
                        hd = res.handedness[k][0].category_name
                        hscore = float(res.handedness[k][0].score)
                        if calib['handedness']['swap']:
                            hd = 'Left' if hd == 'Right' else 'Right'
                    g = pose_map.Geometry.compute(
                        res.hand_landmarks[k], res.hand_world_landmarks[k],
                        calib['extend']['margin_ratio'], 1.0 if hd == 'Right' else -1.0)
                    if float(calib['zero'].get('hand_px0', 0.0)) <= 1e-6:
                        hp = float(g['hand_px'])
                        # 误检（手很小/在画面边缘）不能当基准，否则 size_ratio 会爆、x 被顶到工作空间边界
                        if pose_map.MIN_HAND_PX < hp < 0.60:
                            calib['zero']['hand_px0'] = hp
                            print('[前后] hand_px0 缺失（calib 里是 0），用当前帧手长 %.1f px 作基准'
                                  '（≥%.0fpx 才采）；否则 x 永远不动'
                                  % (hp * frame.shape[1], pose_map.MIN_HAND_PX * 640))
                        elif not st.get('_px0_warn'):
                            st['_px0_warn'] = True
                            print('[前后] 手太小/误检（%.1f px），先不采基准；把手凑近摄像头再试'
                                  % (hp * frame.shape[1]))
                    stats['hand'] += 1
                    stats['hand_px'] += float(g['hand_px']) * frame.shape[1]
                if g is not None:
                    pose, flags = mapper.map(g, dt)
                    g['palm_deg'] = pose.get('palm_deg', 0.0)
                    auto_g = az.feed(g, now)              # 握拳两次 -> 张掌自动标零
                    if auto_g is not None:
                        z = pose_map.apply_zero(calib, auto_g)
                        mapper.reset_smoothing()
                        print('[自动标定] 零位已更新 u %.4f v %.4f 手长 %.1fpx theta %+.1f°'
                              '（按 w 存盘）' % (z['u'], z['v'],
                                                z['hand_px0'] * frame.shape[1], z['theta_deg']))
                else:
                    az.feed(None, now)
                    pose, flags = mapper.hold(), []
                if freeze and frozen_pose is not None:
                    pose, flags = dict(frozen_pose), []
                    g = None
                elif g is not None:
                    frozen_pose = dict(pose)
                # 手长比这一路噪声大（误检会让 size_ratio 爆掉、把 x 顶到边界），
                # 把 x 相对起始点的偏移限在 ±x_range 内（默认 ±8cm）
                xs0 = float(calib['arm_start']['xyz'][0])
                pose['x'] = max(xs0 - args.x_range, min(xs0 + args.x_range, float(pose['x'])))
                seq += 1
                line = pose_map.encode_pose1(seq, 1, pose)
                st['freeze'] = freeze
                flash_txt = az.flash if now < az.flash_until else ''
                if full_hud:
                    W0 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or frame.shape[1]
                    H0 = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or frame.shape[0]
                    left = pose_map.draw(frame.copy(), res, g, pose, line, flags, calib, hd,
                                         hscore, painter, st['fps'], W0, H0, frozen=freeze,
                                         flash=flash_txt)
                else:
                    left = panel_camera_fast(frame, res, g, pose, flags, st, hd)
                    left = blit(left, chips.get(flash_txt))
                prof['hud'] += time.perf_counter() - _t2
            else:
                got = None
                while True:
                    try:
                        row = next(row_iter)
                    except StopIteration:
                        if args.replay_loop:
                            row_iter = replay_rows(args.replay)
                            rp_t0 = rp_first_ms = None
                            continue
                        row = None
                    except Exception:
                        row = None
                    if row is None:
                        break
                    p = row_to_pose(row)
                    if p is not None:
                        got = (row, p)
                        break
                if got is None:
                    print('[结束] 回放结束')
                    break
                row, pose = got
                pose = dict(pose)
                flags = list(row.get('clamp_flags') or [])
                line = row.get('pose1_line') or pose_map.encode_pose1(row.get('seq', 0), 1, pose)
                ts_ms = int(row.get('ts_ms', 0))
                st['freeze'] = freeze
                if freeze and frozen_pose is not None:
                    pose = dict(frozen_pose)
                else:
                    frozen_pose = dict(pose)
                if rp_t0 is None:
                    rp_t0, rp_first_ms = time.time(), ts_ms
                else:
                    span = max(0, ts_ms - rp_first_ms) / max(1e-6, args.replay_speed) / 1000.0
                    slack = (rp_t0 + span) - time.time()
                    if slack > 0:
                        time.sleep(min(slack, 0.25))

            # ================= IK =================
            _ta = time.perf_counter()
            pose_ik = fix_yaw(pose)
            Tt_cmd = kin.make_target(pose_ik['x'], pose_ik['y'], pose_ik['z'],
                                     pose_ik['roll'], pose_ik['pitch'], pose_ik['yaw'])
            pose_solve, over_m = clamp_reach(pose_ik, args.clamp_reach, args.reach_margin)
            q, r, ik_ms, Tt, ok, conv = ik_step(q, pose_solve, args.max_steps,
                                               args.stall, args.accept_pos, args.accept_rot)
            stats['ik_ok'] += 1 if conv else 0
            stats['ik_acc'] += 1 if (ok and not conv) else 0
            stats['ik_hold'] += 0 if ok else 1
            stats['ik_ms'] += ik_ms
            ee = kin.fk(q)[:3, 3]
            prof['ik'] += time.perf_counter() - _ta
            if not trail or float(np.linalg.norm(trail[-1] - ee)) > 1e-4:
                trail.append(ee.copy())
                if len(trail) > 400:
                    del trail[:len(trail) - 400]

            # ================= 渲染 =================
            _tr = time.perf_counter()
            scene.build_static(painter, ws_lo, ws_hi)
            scene.begin()
            draw_trail(scene, trail[-160:])
            draw_target_ghost(scene, Tt_cmd, pose['gripper'])
            T_ee = draw_arm(scene, q, pose['gripper'])
            scene.frame(T_ee, 0.04, thick=2)
            draw_ee_marker(scene, T_ee)
            scene.end()

            frames += 1
            if frames % 8 == 0 or frames == 1:
                st['fps'] = frames / max(1e-6, time.time() - t_loop0)
            st.update(n=row.get('seq', frames) if not live else frames,
                      code=r['code'], it=r['iterations'], hold=not ok, conv=conv,
                      pos_err=r['pos'], rot_err=r['rot'], ik_ms=ik_ms,
                      target=(pose['x'], pose['y'], pose['z'], pose['roll'],
                              pose['pitch'], pose['yaw']),
                      ee=ee, gripper=pose['gripper'],
                      lim=kin.limit_violations(q), ts=ts_ms,
                      last_hand_px=(float(g['hand_px']) if g is not None
                                    else st.get('last_hand_px', 0.0)),
                      note=('目标超出可达球 %.0f mm，已夹到边界' % (over_m * 1000.0)
                            if over_m > 1e-4 else ''))
            hud_3d_dynamic(scene, st)
            right = scene.flush()
            if left is None:
                left = panel_offline(args.panel_w, args.height, st, pose, line, flags)
            left = _fit_height(left, args.panel_h)
            if dump is None or dump.shape != (args.panel_h, left.shape[1] + right.shape[1], 3):
                dump = np.zeros((args.panel_h, left.shape[1] + right.shape[1], 3), np.uint8)
            dump[:, :left.shape[1]] = left
            dump[:, left.shape[1]:] = right
            prof['render'] += time.perf_counter() - _tr
            stats['frames'] += 1

            if args.max_frames and frames >= args.max_frames:
                print('[结束] 到达 --max-frames', args.max_frames)
                break

            if not args.no_window:
                _ts = time.perf_counter()
                cv2.imshow(win, dump)
                k = cv2.waitKey(1) & 0xFF
                ck = con_keys.get()
                prof['show'] += time.perf_counter() - _ts
                if ck != -1:
                    k = ck
                if k not in (-1, 255) and not key_handle(k):
                    break
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                    print('  窗口已关闭')
                    break
            else:
                k = con_keys.get()
                if k not in (-1, 255) and not key_handle(k):
                    break

            if writer is not None:
                writer.write(dump)
            if not args.quiet and frames % max(1, args.print_every) == 0:
                npr = max(1, args.print_every)
                print('[%05d] %s  IK %s it=%d pos=%.3fmm rot=%.3f° %.2fms | 耗时/帧 '
                      '读%.1f 检%.1f 左%.1f IK%.1f 渲%.1f 显示%.1f ms | 手长%.0fpx' % (
                          frames, line, 'OK    ' if conv else ('ACCEPT' if ok else 'HOLD  '),
                          r['iterations'], r['pos'] * 1000.0, math.degrees(r['rot']), ik_ms,
                          prof['read'] * 1000.0 / npr, prof['detect'] * 1000.0 / npr,
                          prof['hud'] * 1000.0 / npr, prof['ik'] * 1000.0 / npr,
                          prof['render'] * 1000.0 / npr, prof['show'] * 1000.0 / npr,
                          stats['hand_px'] / npr))
                for _k in prof:
                    prof[_k] = 0.0
                stats['hand_px'] = 0.0
    finally:
        el = time.time() - t_loop0
        if cap is not None:
            cap.release()
        if writer is not None:
            writer.release()
        pose_map.close_quietly(landmarker)
        if not args.no_window:
            cv2.destroyAllWindows()

    if args.dump_frame and dump is not None:
        pth = Path(args.dump_frame)
        pth.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(pth), dump)
        print('[输出] 末帧 ->', pth)

    n = stats['frames']
    print('\n[统计] %d 帧 / %.1fs = %.1f fps；IK 平均 %.2f ms/次，收敛 %d / 容差内 %d / 沿用 %d' % (
        n, el, n / max(1e-6, el), stats['ik_ms'] / max(1, n),
        stats['ik_ok'], stats['ik_acc'], stats['ik_hold']))
    (OUT_DIR / ('sim_session_%s.json' % time.strftime('%Y%m%d_%H%M%S'))).write_text(
        json.dumps(dict(frames=n, seconds=round(el, 2), fps=round(n / max(1e-6, el), 1),
                        ik_ms_avg=round(stats['ik_ms'] / max(1, n), 3),
                        ik_ok=stats['ik_ok'], ik_acc=stats['ik_acc'],
                        ik_hold=stats['ik_hold'], replay=args.replay,
                        source=None if not live else str(args.source),
                        calib=args.calib), ensure_ascii=False, indent=1), encoding='utf-8')
    return 0


def _fit_height(img, h):
    if img.shape[0] == h:
        return img
    w = max(1, int(round(img.shape[1] * h / max(1, img.shape[0]))))
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)


if __name__ == '__main__':
    _code = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(_code if isinstance(_code, int) else 0)
