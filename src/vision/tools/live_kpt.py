# -*- coding: utf-8 -*-
"""stage1 关键点模型实时演示（train-pi 环境，ONNX/ORT CPU，无需 torch、无需 mediapipe）。

**口径（重要）**：模型只做"给定框内的 21 点回归"，**不做全帧定位**，而且 presence 头只喂过
正样本（没学过"没有手"），所以它对任何输入框都会自信地画出一只手——在脸上、墙上都一样。
因此本脚本**不用自动找手**，改成：鼠标拖一个框把手框住（或 --roi 直接给），之后靠跟踪跟住。
全帧定位归阶段一检测器，尚未训练。

预处理与训练严格一致（tools/self_video_cache.py + train_kpt.CacheDS）：
  原生分辨率方窗 -> INTER_AREA 到 256² -> BGR（不转 RGB）/127.5-1.0 -> NCHW

用法（train-pi）：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/live_kpt.py --cam 0
  D:/Anaconda3/envs/train-pi/python.exe -u tools/live_kpt.py --video x.mp4 --roi 200,700,600
  D:/Anaconda3/envs/train-pi/python.exe -u tools/live_kpt.py --video x.mp4 --headless --max-frames 60 --roi 200,700,600
按键：鼠标拖框=给初值 | q/ESC 退出 | r 重选框 | t 开关跟踪 | p 冻结 | c 存图 | m 换模型 | [ ] w a s d 手调
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sw'))
import pose_map  # noqa: E402

SELF = ROOT / 'results/stage1/self_run1/best.onnx'
RUN4 = ROOT / 'results/stage1/sanity_run4_roi_hm16/best.onnx'
OUTDIR = ROOT / 'results/stage1/live_cam'
WIN = 'kpt live (stage1)'
MARGIN_RATIO = 0.15          # 与 eval_kpt.ext_bits 同口径
SCALE_MED = 0.42             # 训练缓存 hand_len/ROI_side 中位数

_drag = {'on': False, 'a': None, 'b': None, 'res': None}
DISP = {'s': 1.0}          # 显示缩放：鼠标坐标要除回去


def _on_mouse(ev, x, y, flags, _):
    s = DISP['s']
    x, y = int(x * s), int(y * s)
    if ev == cv2.EVENT_LBUTTONDOWN:
        _drag['on'], _drag['a'], _drag['b'] = True, (x, y), (x, y)
    elif ev == cv2.EVENT_MOUSEMOVE and _drag['on']:
        _drag['b'] = (x, y)
    elif ev == cv2.EVENT_LBUTTONUP and _drag['on']:
        _drag['on'] = False
        a, b = _drag['a'], _drag['b']
        if a and b and abs(b[0] - a[0]) > 40 and abs(b[1] - a[1]) > 40:
            _drag['res'] = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0,
                            float(max(b[0] - a[0], b[1] - a[1])))


def theta_deg(p):
    return float(np.degrees(np.arctan2(p[9, 1] - p[0, 1], p[9, 0] - p[0, 0])))


def ext_bits(p, hp):
    m = MARGIN_RATIO * hp
    return tuple(int(np.linalg.norm(p[t] - p[0]) > np.linalg.norm(p[q] - p[0]) + m)
                 for t, q in zip(pose_map.TIP, pose_map.PIP))


def crop_square(img, cx, cy, side):
    H, W = img.shape[:2]
    side = int(max(16, min(side, 4 * max(H, W))))
    x0, y0 = int(round(cx - side / 2)), int(round(cy - side / 2))
    pl, pt = max(0, -x0), max(0, -y0)
    pr, pb = max(0, x0 + side - W), max(0, y0 + side - H)
    im = img
    if pl or pr or pt or pb:
        im = cv2.copyMakeBorder(img, pt, pb, pl, pr, cv2.BORDER_REPLICATE)
    c = im[y0 + pt:y0 + pt + side, x0 + pl:x0 + pl + side]
    return c, (x0, y0, side)


def preprocess(crop):
    x = cv2.resize(crop, (256, 256), interpolation=cv2.INTER_AREA)
    x = x.astype(np.float32) / 127.5 - 1.0
    return x.transpose(2, 0, 1)[None]


def infer(frame, roi, sess):
    crop, (x0, y0, sd) = crop_square(frame, roi[0], roi[1], roi[2])
    c, _, _ = sess.run(None, {'img': preprocess(crop)})
    p = np.asarray(c, np.float32).reshape(21, 2)
    pts = np.stack([x0 + p[:, 0] * sd, y0 + p[:, 1] * sd], axis=1)
    return pts, crop


def draw_hand(img, pts, col):
    for i, j in pose_map.HAND_LINKS:
        cv2.line(img, tuple(np.int32(pts[i])), tuple(np.int32(pts[j])), col, 2, cv2.LINE_AA)
    for k, p in enumerate(pts):
        r = 5 if k in pose_map.TIP else 3
        cv2.circle(img, tuple(np.int32(p)), r, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, tuple(np.int32(p)), r, col, 1, cv2.LINE_AA)


def _show(img, ds):
    return cv2.resize(img, None, fx=ds, fy=ds, interpolation=cv2.INTER_AREA) if ds < 0.999 else img


def panel(h, crop, txt):
    cv = np.full((h, 300, 3), 28, np.uint8)
    if crop is not None:
        cv[8:200, 54:246] = cv2.resize(crop, (192, 192), interpolation=cv2.INTER_NEAREST)
        cv2.putText(cv, 'model input 256x256', (40, 222), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (170, 170, 170), 1, cv2.LINE_AA)
    y = 252
    for s in txt:
        col = (0, 0, 255) if s.startswith('!') else (0, 220, 0)
        cv2.putText(cv, s.lstrip('!'), (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.44, col, 1, cv2.LINE_AA)
        y += 22
    return cv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', type=int, default=0)
    ap.add_argument('--video', default=None)
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--onnx', default=str(SELF))
    ap.add_argument('--roi', default=None, help='初值框 "cx,cy,side"（不给就鼠标拖框）')
    ap.add_argument('--width', type=int, default=1920,
                    help='采集分辨率（训练用的是 1080p 录像，所以默认 1080p）')
    ap.add_argument('--height', type=int, default=1080)
    ap.add_argument('--disp-scale', type=float, default=0.0, help='窗口显示缩小倍数，0=自适应')
    ap.add_argument('--no-track', action='store_true')
    ap.add_argument('--alpha', type=float, default=0.45)
    args = ap.parse_args()

    import onnxruntime as ort
    sess = ort.InferenceSession(args.onnx, sess_options=ort.SessionOptions(),
                               providers=['CPUExecutionProvider'])
    print(f'[模型] {args.onnx}', flush=True)

    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print('[错误] 打不开视频源', flush=True)
        return 2
    ok, f0 = cap.read()
    if not ok:
        print('[错误] 读不到帧', flush=True)
        return 2
    H, W = f0.shape[:2]
    ds = args.disp_scale if args.disp_scale > 0 else min(1.0, 1400.0 / W)
    DISP['s'] = ds

    roi = None
    if args.roi:
        cx, cy, sd = [float(x) for x in args.roi.split(',')]
        roi = [cx, cy, sd]
    if not args.headless:
        cv2.namedWindow(WIN)
        cv2.setMouseCallback(WIN, _on_mouse)

    track = not args.no_track
    frozen = None
    pts_s = None
    crop_last = None
    n, n_bad, lock_bad, fps_t0 = 0, 0, 0, time.time()
    print(f'[源] {"video " + args.video if args.video else "cam " + str(args.cam)}  {W}x{H}  '
          f'跟踪={track}  初值={"无（鼠标拖框）" if roi is None else args.roi}', flush=True)

    while True:
        if _drag['res'] is not None:
            roi = list(_drag['res'])
            _drag['res'], pts_s, frozen = None, None, None
            print(f'[重设框] {roi[0]:.0f},{roi[1]:.0f},{roi[2]:.0f}', flush=True)

        if frozen is None:
            ok, frame = cap.read()
            if not ok:
                break
        else:
            frame = frozen

        vis = frame.copy()
        if roi is None:
            cv2.putText(vis, 'DRAG a box around your hand', (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2, cv2.LINE_AA)
            cv2.putText(vis, 'use mouse to drag / r to reset', (30, 85),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 1, cv2.LINE_AA)
            if not args.headless:
                cv2.imshow(WIN, _show(np.hstack([vis, panel(H, None, ['waiting for ROI'])]), ds))
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), 27):
                    break
                if k == ord('p'):
                    frozen = None if frozen is not None else frame
            continue

        pts, crop = infer(frame, roi, sess)
        n += 1
        hp = float(np.linalg.norm(pts[9] - pts[0]))
        # 自跟踪：框心跟预测 bbox 心（EMA），框边按 掌长/0.42 反推但限速 ±5%/帧
        if track and hp > 0.05 * min(H, W):
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            bcx, bcy = (lo + hi) / 2.0
            a = args.alpha
            side_t = float(np.clip(hp / SCALE_MED, roi[2] * 0.95, roi[2] * 1.05))
            side_t = float(np.clip(side_t, 0.25 * min(H, W), 1.2 * min(H, W)))
            roi = [float(np.clip(roi[0] + a * (bcx - roi[0]), 0, W)),
                   float(np.clip(roi[1] + a * (bcy - roi[1]), 0, H)),
                   roi[2] + a * (side_t - roi[2])]
            crop_last = crop
            pts, crop = infer(frame, roi, sess)      # 第二遍：新框里重新出点
            hp = float(np.linalg.norm(pts[9] - pts[0]))
            if 0.15 < hp / roi[2] < 0.95:
                lock_bad = 0
            else:
                lock_bad += 1
        crop_last = crop

        # 点位平滑 + 突跳门控
        bad = False
        if pts_s is not None:
            jump = float(np.linalg.norm(pts - pts_s, axis=1).mean())
            if jump > 0.45 * hp:
                bad = True
                pts_s = pts_s * 0.7 + pts * 0.3
            else:
                pts_s = pts_s + args.alpha * (pts - pts_s)
        else:
            pts_s = pts.copy()
        n_bad += int(bad)
        pn = (pts_s - np.array([roi[0], roi[1]], np.float32)) / roi[2]

        fps = n / max(1e-6, time.time() - fps_t0)
        txt = [f'model  {Path(args.onnx).parent.name}',
               f'scale  {hp / roi[2]:.2f} (train med 0.42)',
               f'theta  {theta_deg(pn):6.1f} deg',
               f'ext  {"".join(str(b) for b in ext_bits(pn, hp / roi[2]))}',
               f'fps {fps:5.1f}   {n} frames',
               f'roi {roi[2]:.0f}px @({roi[0]:.0f},{roi[1]:.0f})',
               f'track {"ON" if track else "OFF"}  bad {n_bad}',
               'q quit  r reseed  t track',
               'p freeze  c snap  m model',
               'drag=reseed  []wasd=size/move']
        if bad:
            txt.insert(0, '!姿态突跳，已抑制')
        if lock_bad > 6:
            txt.insert(0, '!跟丢：请鼠标重拖或按 r')
        if args.video:
            txt.insert(0, Path(args.video).name)

        cv2.rectangle(vis, (int(roi[0] - roi[2] / 2), int(roi[1] - roi[2] / 2)),
                      (int(roi[0] + roi[2] / 2), int(roi[1] + roi[2] / 2)), (60, 160, 255), 1)
        draw_hand(vis, pts_s, (140, 140, 140) if bad else (0, 0, 255))

        if args.headless:
            vis_all = np.hstack([vis, panel(H, crop_last, txt)])
            if n <= 40:
                OUTDIR.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(OUTDIR / f'headless_{n:03d}.png'), vis_all)
            if n % 10 == 0:
                print(f'  #{n} theta={theta_deg(pn):+.1f} scale={hp/roi[2]:.2f} '
                      f'roi={roi[2]:.0f} fps={fps:.1f}', flush=True)
            if args.max_frames and n >= args.max_frames:
                break
            continue

        cv2.imshow(WIN, _show(np.hstack([vis, panel(H, crop_last, txt)]), ds))
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        elif k == ord('r'):
            roi, pts_s, frozen, lock_bad = None, None, None, 0
        elif k == ord('t'):
            track = not track
        elif k == ord('p'):
            frozen = None if frozen is not None else frame
        elif k == ord('['):
            roi[2] *= 0.9
        elif k == ord(']'):
            roi[2] *= 1.1
        elif k == ord('w'):
            roi[1] -= 20
        elif k == ord('d'):
            roi[0] += 20
        elif k == ord('a'):
            roi[0] -= 20
        elif k == ord('x'):
            roi[1] += 20
        elif k == ord('m'):
            alt = RUN4 if 'self_run1' in args.onnx else SELF
            sess = ort.InferenceSession(str(alt), sess_options=ort.SessionOptions(),
                                        providers=['CPUExecutionProvider'])
            args.onnx = str(alt)
            print(f'[切模型] {alt.parent.name}', flush=True)
        elif k == ord('c'):
            OUTDIR.mkdir(parents=True, exist_ok=True)
            fp = OUTDIR / f'snap_{time.strftime("%H%M%S")}.png'
            cv2.imwrite(str(fp), np.hstack([vis, panel(H, crop_last, txt)]))
            print(f'[存图] {fp}', flush=True)

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()
    print(f'[结束] {n} 帧，突跳抑制 {n_bad} 次', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
