# -*- coding: utf-8 -*-
"""stage1 关键点模型实时演示（train-pi 环境，ONNX/ORT CPU，无需 torch、无需 mediapipe）。

**口径（重要）**：模型只做“给定框内的 21 点回归”，**不做全帧定位**。
Stage1 自录数据训练后 presence 头已有判别力（带过负样本），但对“框里没手”以外的
情况（框偏一半、脸）仍不可靠，所以本脚本**不用自动找手**。

**ROI 默认钉在画面正中央、固定不动**（`--roi-side` 控制边长，默认短边 0.6）。
遥控棒场景里手就在固定位置，固定框最简单也最好拷到板子上：不需要任何 ROI 控制器。
需要跟随时加 `--track`（鼠标拖框给初值，之后 EMA 跟随）。

预处理与训练严格一致（tools/self_video_cache.py + train_kpt.CacheDS）：
  原生分辨率方窗 -> INTER_AREA 到模型输入尺寸（192²）-> BGR（不转 RGB）/127.5-1.0 -> NCHW

用法（train-pi）：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/live_kpt.py --cam 0
  D:/Anaconda3/envs/train-pi/python.exe -u tools/live_kpt.py --cam 0 --roi-side 0.5
  D:/Anaconda3/envs/train-pi/python.exe -u tools/live_kpt.py --video x.mp4 --roi 200,700,600
  D:/Anaconda3/envs/train-pi/python.exe -u tools/live_kpt.py --video x.mp4 --headless --max-frames 60 --roi 200,700,600
按键：q/ESC 退出 | r 回中央框 | t 开关跟随 | p 冻结 | c 存图 | m 换模型 | [ ] w a s d 手调
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

# 模型 -> (onnx, 训练缓存里 hand_len/ROI_side 的中位数)
MODELS = {'v5i8': (ROOT / 'results/stage1/self_run5_192/int8/best_hm_int8.onnx', 0.35),  # ★ 部署机位+全姿态 int8 热图
          'v5': (ROOT / 'results/stage1/self_run5_192/best.onnx', 0.35),  # 同上 fp32 坐标出口
          'v4i8': (ROOT / 'results/stage1/self_run4_192/int8_hm/best_hm_int8.onnx', 0.36),  # int8 热图
          'v4': (ROOT / 'results/stage1/self_run4_192/best.onnx', 0.36),  # v1+v2 合并 @192²
          'v3': (ROOT / 'results/stage1/self_run3/best.onnx', 0.36),   # v1+v2 合并 @256²
          'v2': (ROOT / 'results/stage1/self_run2/best.onnx', 0.30),   # 近远+负样本
          'v1': (ROOT / 'results/stage1/self_run1/best.onnx', 0.42),   # 第一支录像
          'run4': (ROOT / 'results/stage1/sanity_run4_roi_hm16/best.onnx', 0.42)}
SELF = MODELS['v5i8'][0]
OUTDIR = ROOT / 'results/stage1/live_cam'
WIN = 'kpt live (stage1)'
MARGIN_RATIO = 0.15          # 与 eval_kpt.ext_bits 同口径
SCALE_MED = 0.42             # --scale-med 0 时按模型自动查


def scale_of(path):
    for p, s in MODELS.values():
        if Path(p) == Path(path):
            return s
    return SCALE_MED

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


def preprocess(crop, size=256):
    x = cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)
    x = x.astype(np.float32) / 127.5 - 1.0
    return x.transpose(2, 0, 1)[None]


def in_size_of(sess):
    s = sess.get_inputs()[0].shape
    return int(s[2]) if len(s) == 4 else 256


def decode_hm(z):
    """(21,hm,hm) 热图 logits -> (21,2)。softmax 在主机算，int8 模型必须走这条路。"""
    hm = z.shape[-1]
    Z = np.asarray(z, np.float32).reshape(21, hm * hm)
    e = np.exp(Z - Z.max(axis=1, keepdims=True))
    w = (e / e.sum(axis=1, keepdims=True)).reshape(21, hm, hm)
    g = (np.arange(hm, dtype=np.float32) + 0.5) / hm
    return np.stack([(w * g[None, None, :]).sum(axis=(1, 2)),
                     (w * g[None, :, None]).sum(axis=(1, 2))], axis=1)


def infer(frame, roi, sess, size=256):
    crop, (x0, y0, sd) = crop_square(frame, roi[0], roi[1], roi[2])
    c, l_pres, l_hand = sess.run(None, {'img': preprocess(crop, size)})
    c = np.asarray(c, np.float32)
    p = decode_hm(c[0]) if c.ndim == 4 else c.reshape(21, 2)
    pts = np.stack([x0 + p[:, 0] * sd, y0 + p[:, 1] * sd], axis=1)
    pv = 1.0 / (1.0 + np.exp(-float(np.asarray(l_pres).ravel()[0])))
    lh = np.asarray(l_hand, np.float32).ravel()
    return pts, pv, int(lh.argmax()), crop


def draw_hand(img, pts, col):
    for i, j in pose_map.HAND_LINKS:
        cv2.line(img, tuple(np.int32(pts[i])), tuple(np.int32(pts[j])), col, 2, cv2.LINE_AA)
    for k, p in enumerate(pts):
        r = 5 if k in pose_map.TIP else 3
        cv2.circle(img, tuple(np.int32(p)), r, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(img, tuple(np.int32(p)), r, col, 1, cv2.LINE_AA)


def _show(img, ds):
    return cv2.resize(img, None, fx=ds, fy=ds, interpolation=cv2.INTER_AREA) if ds < 0.999 else img


def panel(h, crop, txt, ins=256):
    cv = np.full((h, 300, 3), 28, np.uint8)
    if crop is not None:
        cv[8:200, 54:246] = cv2.resize(crop, (192, 192), interpolation=cv2.INTER_NEAREST)
        cv2.putText(cv, f'model input {ins}x{ins}', (40, 222), cv2.FONT_HERSHEY_SIMPLEX,
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
    ap.add_argument('--scale-med', type=float, default=0.0,
                    help='训练缓存 hand_len/ROI_side 中位数；0=按模型自动查')
    ap.add_argument('--pres-thr', type=float, default=0.5, help='presence 概率阈值')
    ap.add_argument('--roi', default=None, help='初值框 "cx,cy,side"；不给则居中固定框（--track 时改为鼠标拖框）')
    ap.add_argument('--roi-side', type=float, default=0.6,
                    help='初值/固定框边长占短边的比例（默认 0.6，1080p 即 648 px）')
    # 实测本机 USB2 摄像头只出 YUY2：1080p=5fps / 720p=10fps / 480p=30fps。
    # ROI 归一化后几何输入与采集分辨率无关（都缩到 192²），分辨率只影响锐度，
    # 所以默认用部署端的 640x480@30 保证流畅。
    ap.add_argument('--width', type=int, default=640, help='采集宽度（默认 640，与部署相机一致）')
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--disp-scale', type=float, default=0.0, help='窗口显示缩小倍数，0=自适应')
    ap.add_argument('--track', dest='track', action='store_true', default=True,
                    help='ROI 自动跟随（默认开）')
    ap.add_argument('--no-track', dest='track', action='store_false',
                    help='关掉跟随：框固定不动')
    ap.add_argument('--alpha', type=float, default=0.45)
    ap.add_argument('--start-frame', type=int, default=0, help='视频回放起始帧')
    args = ap.parse_args()
    if args.scale_med <= 0:
        args.scale_med = scale_of(args.onnx)

    import onnxruntime as ort
    so = ort.SessionOptions()
    so.log_severity_level = 3          # 抬掉 int8 图加载时的 Removing initializer 刷屏
    sess = ort.InferenceSession(args.onnx, sess_options=so,
                               providers=['CPUExecutionProvider'])
    print(f'[模型] {args.onnx}', flush=True)
    ins = in_size_of(sess)
    print(f'[模型] 输入 {ins}x{ins}', flush=True)

    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # 1080p 不开 MJPG 只有几帧
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print('[错误] 打不开视频源', flush=True)
        return 2
    if args.start_frame and args.video:
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)
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
    track = args.track
    if roi is None:                     # 无初值就用居中框起步，开了跟随会自己跑过去
        roi = [W / 2.0, H / 2.0, float(args.roi_side) * min(H, W)]
    if not args.headless:
        cv2.namedWindow(WIN)
        cv2.setMouseCallback(WIN, _on_mouse)

    frozen = None
    pts_s = None
    crop_last = None
    hp_buf = []
    side_locked = False if track else True      # 跟随时先采 5 帧自动定框，然后冻结
    n, n_bad, lock_bad, fps_t0 = 0, 0, 0, time.time()
    print(f'[源] {"video " + args.video if args.video else "cam " + str(args.cam)}  {W}x{H}  '
          f'ROI={"跟踪" if track else "固定居中"}  '
          f'框=%s' % (f'{roi[0]:.0f},{roi[1]:.0f},{roi[2]:.0f}' if roi else '待鼠标拖框'), flush=True)

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
                cv2.imshow(WIN, _show(np.hstack([vis, panel(H, None, ['waiting for ROI'], ins)]), ds))
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), 27):
                    break
                if k == ord('p'):
                    frozen = None if frozen is not None else frame
            continue

        pts, pv, hnd, crop = infer(frame, roi, sess, ins)
        n += 1
        hp = float(np.linalg.norm(pts[9] - pts[0]))
        present = pv >= args.pres_thr
        # 跟随：只跟框心（EMA）；框边一次性按 掌长/scale_med 定下后**冻结**
        # 不再拿模型自己的掌长去反推边长——那是个闭环，会双向往跑（见过缩到地板，也见过越跑越大）
        if track and present and hp > 0.05 * min(H, W):
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            bcx, bcy = (lo + hi) / 2.0
            if not side_locked:
                hp_buf.append(hp)
                if len(hp_buf) >= 5:
                    med = float(np.median(hp_buf))
                    roi[2] = float(np.clip(med / args.scale_med,
                                           0.2 * min(H, W), 1.2 * min(H, W)))
                    side_locked = True
                    print(f'[自动定框] 掌长中位 {med:.0f}px / {args.scale_med:.2f} '
                          f'-> 边长 {roi[2]:.0f}px', flush=True)
            a = args.alpha
            roi = [float(np.clip(roi[0] + a * (bcx - roi[0]), 0, W)),
                   float(np.clip(roi[1] + a * (bcy - roi[1]), 0, H)),
                   roi[2]]
            crop_last = crop
            pts, pv, hnd, crop = infer(frame, roi, sess, ins)      # 第二遍：新框里重新出点
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
               f'pres  {pv:.2f} ({"HAND" if present else "NO HAND"})',
               f'scale  {hp / roi[2]:.2f} (train med {args.scale_med:.2f})',
               f'theta  {theta_deg(pn):6.1f} deg',
               f'ext  {"".join(str(b) for b in ext_bits(pn, hp / roi[2]))}',
               f'fps {fps:5.1f}   {n} frames',
               f'roi {roi[2]:.0f}px @({roi[0]:.0f},{roi[1]:.0f})',
               f'ROI {"跟踪" if track else "固定中心"}  bad {n_bad}',
               'q quit  r reseed  t track',
               'p freeze  c snap  m model',
               'drag=reseed  []wasd=size/move']
        if bad:
            txt.insert(0, '!姿态突跳，已抑制')
        if lock_bad > 6:
            txt.insert(0, '!跟丢：请鼠标重拖或按 r')
        if args.video:
            txt.insert(0, Path(args.video).name)

        rect_c = (60, 160, 255)
        cv2.rectangle(vis, (int(roi[0] - roi[2] / 2), int(roi[1] - roi[2] / 2)),
                      (int(roi[0] + roi[2] / 2), int(roi[1] + roi[2] / 2)), rect_c, 1)
        if present:
            draw_hand(vis, pts_s, (140, 140, 140) if bad else (0, 0, 255))
        else:
            cv2.putText(vis, 'NO HAND', (int(roi[0] - roi[2] / 2), int(roi[1] - roi[2] / 2) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        if args.headless:
            vis_all = np.hstack([vis, panel(H, crop_last, txt, ins)])
            if n <= 40:
                OUTDIR.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(OUTDIR / f'headless_{n:03d}.png'), vis_all)
            if n % 10 == 0:
                print(f'  #{n} theta={theta_deg(pn):+.1f} scale={hp/roi[2]:.2f} '
                      f'roi={roi[2]:.0f} fps={fps:.1f}', flush=True)
            if args.max_frames and n >= args.max_frames:
                break
            continue

        cv2.imshow(WIN, _show(np.hstack([vis, panel(H, crop_last, txt, ins)]), ds))
        if n % 30 == 0:                     # 每 30 帧落一条日志，便于事后查框大小/置信度
            print(f'  #{n} pres={pv:.2f} scale={hp/roi[2]:.2f} roi={roi[2]:.0f} '
                  f'theta={theta_deg(pn):+.0f} fps={fps:.1f}', flush=True)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        elif k == ord('r'):
            roi = [W / 2.0, H / 2.0, float(args.roi_side) * min(H, W)]
            pts_s, frozen, lock_bad = None, None, 0
            hp_buf, side_locked = [], not track
        elif k == ord('t'):
            track = not track
        elif k == ord('p'):
            frozen = None if frozen is not None else frame
        elif k == ord('['):
            roi[2] *= 0.9
            side_locked = True
        elif k == ord(']'):
            roi[2] *= 1.1
            side_locked = True
        elif k == ord('w'):
            roi[1] -= 20
        elif k == ord('d'):
            roi[0] += 20
        elif k == ord('a'):
            roi[0] -= 20
        elif k == ord('x'):
            roi[1] += 20
        elif k == ord('m'):
            names = list(MODELS)
            cur = next((k for k, v in MODELS.items() if str(v[0]) == args.onnx), None)
            nxt = names[(names.index(cur) + 1) % len(names)] if cur else names[0]
            alt, args.scale_med = MODELS[nxt]
            sess = ort.InferenceSession(str(alt), sess_options=so,
                                        providers=['CPUExecutionProvider'])
            ins = in_size_of(sess)
            args.onnx = str(alt)
            print(f'[切模型] {alt.parent.name}', flush=True)
        elif k == ord('c'):
            OUTDIR.mkdir(parents=True, exist_ok=True)
            fp = OUTDIR / f'snap_{time.strftime("%H%M%S")}.png'
            cv2.imwrite(str(fp), np.hstack([vis, panel(H, crop_last, txt, ins)]))
            print(f'[存图] {fp}', flush=True)

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()
    print(f'[结束] {n} 帧，突跳抑制 {n_bad} 次', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
