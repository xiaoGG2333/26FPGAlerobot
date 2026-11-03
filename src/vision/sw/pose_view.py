# -*- coding: utf-8 -*-
"""极简手部观察窗：只画骨架 + 相对坐标 + 相对手腕旋转，不叠大面板。

和 sw/pose2arm.py 用同一套视觉/标定/映射（pm.Geometry -> pm.PoseMapper），
所以这里的数字和 pose2arm HUD 左上那三行完全同源：

    相对 X/Y/Z = pose 与 calib['arm_start']['xyz'] 的差，单位 mm
    相对旋转   = pose['roll']，单位度（相对标定零位）

区别只有画面：不画 ROI 框、不画左下灵敏度列表、不画中文标签层，
文字用黑描边直接写画面上，不铺半透明底板，尽量少挡。

用法（hand-pi 环境）：
    python sw/pose_view.py                     # 自动挑外接摄像头
    python sw/pose_view.py --source 0
    python sw/pose_view.py --scale 1.5         # 窗口放大 1.5 倍
    python sw/pose_view.py --corner tr         # 文字挪到右上（tl/tr/bl/br）

按键：q 或 ESC 退出   r 存快照   f 切全屏
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sw'))

import cv2                                                    # noqa: E402
import numpy as np                                            # noqa: E402

import pose_map as pm                                         # noqa: E402

OUT_DIR = ROOT / 'results' / 'w3_pose' / 'view'

FONT = cv2.FONT_HERSHEY_SIMPLEX


def put_outline(img, text, org, scale, color, th=1):
    """黑描边白/彩心，不铺底板（底板才是遮挡的主因）。"""
    cv2.putText(img, text, org, FONT, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, FONT, scale, color, th, cv2.LINE_AA)


def text_size(text, scale, th=1):
    (w, h), base = cv2.getTextSize(text, FONT, scale, th)
    return w, h + base


def draw_skeleton(vis, lms, W, H, thick=1):
    """21 点骨架：细线 + 小点，指尖略实。"""
    pts = [(int(l.x * W), int(l.y * H)) for l in lms]
    for a, b in pm.HAND_LINKS:
        cv2.line(vis, pts[a], pts[b], (0, 210, 255), thick, cv2.LINE_AA)
    for i, p in enumerate(pts):
        if i in pm.TIP:
            cv2.circle(vis, p, 3, (60, 230, 60), -1, cv2.LINE_AA)
        elif i == 0:
            cv2.circle(vis, p, 3, (0, 80, 255), -1, cv2.LINE_AA)
        else:
            cv2.circle(vis, p, 2, (255, 255, 255), -1, cv2.LINE_AA)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='auto')
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--calib', default=str(pm.DEFAULT_CALIB))
    ap.add_argument('--model', default=str(pm.DEFAULT_MODEL))
    ap.add_argument('--corner', default='tl', choices=['tl', 'tr', 'bl', 'br'],
                    help='文字位置：tl 左上 / tr 右上 / bl 左下 / br 右下')
    ap.add_argument('--scale', type=float, default=1.0, help='窗口显示放大倍数')
    ap.add_argument('--font', type=float, default=0.55, help='字号')
    ap.add_argument('--no-window', action='store_true')
    ap.add_argument('--max-frames', type=int, default=0)
    args = ap.parse_args()

    calib = pm.load_calib(args.calib)

    # source：auto 要按设备名挑（序号会随插拔平移），和 pose2arm 同一套逻辑
    src = args.source
    if str(src).strip().lower() == 'auto':
        cams = pm.list_cameras()
        if not cams:
            print('[错误] 没枚举到任何摄像头')
            return 9
        for idx, nm, w, h, b in cams:
            print('[摄像头] %d: %-16s %s %dx%d %s'
                  % (idx, nm, '内置' if pm._looks_builtin(nm) else '外接?', w, h, b))
        pick = pm.pick_auto(cams)
        sel = [c for c in cams if c[0] == pick][0]
        print('[摄像头] auto -> %d（%s）' % (pick, sel[1]))
        src = str(pick)

    from mediapipe.tasks.python import vision, BaseOptions
    import mediapipe as mp
    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(args.model)),
        running_mode=vision.RunningMode.VIDEO, num_hands=1,
        min_hand_detection_confidence=0.5, min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5)
    landmarker = vision.HandLandmarker.create_from_options(opts)

    cap = pm.open_capture(str(src), args.width, args.height, args.fps)
    if cap is None or not cap.isOpened():
        print('[错误] 打不开视频源')
        pm.close_quietly(landmarker)
        return 9
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height
    print('[采集] %dx%d 源=%s' % (W, H, src))

    mapper = pm.PoseMapper(calib)
    sx, sy, sz = [float(v) for v in calib['arm_start']['xyz']]
    win = 'pose_view'
    t0 = time.monotonic()
    last_ts = 0
    n = 0
    fps = 0.0
    full = False
    vis = None
    hold_txt = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'size': 1.0}

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            n += 1
            now = time.monotonic()
            ts = max(last_ts + 1, int((now - t0) * 1000))
            last_ts = ts

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_img = mp.Image(image_format=mp.ImageFormat.SRGB,
                              data=np.ascontiguousarray(rgb))
            res = landmarker.detect_for_video(mp_img, ts)

            dt = 1.0 / args.fps if mapper.last_t is None else (now - mapper.last_t)
            mapper.last_t = now

            g = None
            if res.hand_landmarks:
                hd = 'Right'
                if res.handedness:
                    hd = res.handedness[0][0].category_name
                    if calib['handedness']['swap']:
                        hd = 'Left' if hd == 'Right' else 'Right'
                g = pm.Geometry.compute(res.hand_landmarks[0],
                                        res.hand_world_landmarks[0],
                                        calib['extend']['margin_ratio'],
                                        1.0 if hd == 'Right' else -1.0)

            if g is not None:
                if float(calib['zero'].get('hand_px0', 0.0)) <= 1e-6:
                    hp = float(g['hand_px'])
                    if pm.MIN_HAND_PX < hp < 0.60:
                        calib['zero']['hand_px0'] = hp
                pose, flags = mapper.map(g, dt)
                hold_txt.update({
                    'x': (pose['x'] - sx) * 1000.0,
                    'y': (pose['y'] - sy) * 1000.0,
                    'z': (pose['z'] - sz) * 1000.0,
                    'roll': math.degrees(pose['roll']),
                    'size': float(getattr(mapper, 'size_ratio', 1.0)),
                })
            else:
                pose, flags = mapper.hold(), ['丢手保持']

            if n % 10 == 0 or n == 1:
                fps = n / max(1e-6, time.monotonic() - t0)

            # ---------------- 画面 ----------------
            vis = frame.copy()
            if g is not None:
                draw_skeleton(vis, res.hand_landmarks[0], W, H)
            elif vis is not None:
                pass

            fs, th = args.font, 1
            l1 = 'X%+7.1f  Y%+7.1f  Z%+7.1f mm' % (hold_txt['x'], hold_txt['y'],
                                                     hold_txt['z'])
            l2 = 'ROLL %+6.1f deg   size %.2f   %.0f fps' % (
                hold_txt['roll'], hold_txt['size'], fps)
            if g is None:
                l2 = 'NO HAND   last: ' + l2

            w1, h1 = text_size(l1, fs, th)
            w2, h2 = text_size(l2, fs, th)
            pad = int(8 * args.scale)
            if args.corner[0] == 't':
                y1 = pad + h1
            else:
                y1 = H - pad - h2 - h1
            y2 = y1 + h2 + int(4 * args.scale)
            if args.corner[1] == 'l':
                x = pad + 2
            else:
                x = W - max(w1, w2) - pad

            put_outline(vis, l1, (x, y1), fs, (0, 255, 255), th)
            put_outline(vis, l2, (x, y2), fs, (120, 255, 120) if g is not None
                        else (140, 140, 140), th)

            if not args.no_window:
                show = vis
                if args.scale != 1.0:
                    show = cv2.resize(vis, None, fx=args.scale, fy=args.scale,
                                      interpolation=cv2.INTER_NEAREST)
                cv2.imshow(win, show)
                k = cv2.waitKey(1) & 0xFF
                if k in (27, ord('q')):
                    break
                elif k == ord('r'):
                    OUT_DIR.mkdir(parents=True, exist_ok=True)
                    p = OUT_DIR / ('view_%s.png' % time.strftime('%Y%m%d_%H%M%S'))
                    cv2.imwrite(str(p), vis)
                    print('[快照]', p)
                elif k == ord('f'):
                    full = not full
                    cv2.setWindowProperty(win, cv2.WND_PROP_FULLSCREEN,
                                          cv2.WINDOW_FULLSCREEN if full
                                          else cv2.WINDOW_NORMAL)
            if args.max_frames and n >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        pm.close_quietly(landmarker)      # 直接 close() 会挂 ~40 s
        print('[结束] %d 帧，%.1f fps   最后一帧：X%+.1f Y%+.1f Z%+.1f mm  ROLL%+.1f°'
              % (n, fps, hold_txt['x'], hold_txt['y'], hold_txt['z'], hold_txt['roll']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
