"""离线自检：对图片目录跑 MediaPipe HandLandmarker(IMAGE 模式)，出标注 contact sheet。

用途（不依赖摄像头、不需要主人配合）：
  1. 确认 HandLandmarker 在本机的 21 点/手性/世界坐标输出正常
  2. 用 HaGRID 各类别图片核对"伸指数"判别（张开=5 / 闭合=0 / like=1 / ok=3 / peace=2）
  3. 产出标注图给主人看（不能只报数字）

例：
  python -u sw/pose_sheet.py --classes palm fist like ok peace no_gesture --per-class 6
  python -u sw/pose_sheet.py --images some_dir --per-class 12 --out results/w3_pose/sheet_x.png
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'sw'))
from pose_map import (DEFAULT_MODEL, DEFAULT_OUT, HAND_LINKS, TIP, PALM_IDX,  # noqa: E402
                      Geometry, make_painter, clamp, close_quietly)

EXPECT = {'palm': 5, 'fist': 0, 'like': 1, 'ok': 3, 'peace': 2, 'no_gesture': None,
          'stop': None, 'rock': None}
EXTS = '.jpg .jpeg .png .bmp .webp'.split()


def list_images(d, k):
    fs = sorted([p for p in Path(d).iterdir() if p.suffix.lower() in EXTS])
    if len(fs) <= k:
        return fs
    idx = np.linspace(0, len(fs) - 1, k).astype(int)
    return [fs[i] for i in idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default=str(ROOT / 'model' / 'data' / 'hagrid_subset'))
    ap.add_argument('--classes', nargs='*', default=['palm', 'fist', 'like', 'ok', 'peace', 'no_gesture'])
    ap.add_argument('--images', default=None, help='单一目录（覆盖 --root/--classes）')
    ap.add_argument('--per-class', type=int, default=6)
    ap.add_argument('--model', default=str(DEFAULT_MODEL))
    ap.add_argument('--out', default=None)
    ap.add_argument('--tile', type=int, default=200)
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions

    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=args.model),
        running_mode=vision.RunningMode.IMAGE, num_hands=1)
    lm = vision.HandLandmarker.create_from_options(opts)
    painter = make_painter()

    items = []
    if args.images:
        for p in list_images(args.images, args.per_class):
            items.append(('images', p))
    else:
        for c in args.classes:
            d = Path(args.root) / c
            if not d.is_dir():
                print(f'[跳过] 无目录 {d}')
                continue
            for p in list_images(d, args.per_class):
                items.append((c, p))
    if not items:
        print('[错误] 没找到图片')
        return 2

    T = args.tile
    rows, per_row = [], max(1, len(items))
    per_row = min(len(items), 6)
    tiles, stats = [], {}
    t0 = time.time()
    for cls, path in items:
        bgr = cv2.imread(str(path))
        if bgr is None:
            continue
        rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        res = lm.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
        h, w = bgr.shape[:2]
        k = T / max(h, w)
        tile = cv2.resize(bgr, (int(round(w * k)), int(round(h * k))), interpolation=cv2.INTER_AREA)
        canvas = np.zeros((T + 34, T, 3), np.uint8)
        y0, x0 = (T - tile.shape[0]) // 2, (T - tile.shape[1]) // 2
        canvas[y0:y0 + tile.shape[0], x0:x0 + tile.shape[1]] = tile

        texts, ok_msg, col = [], '未检出', (150, 150, 150)
        hd = '-'
        if res.hand_landmarks:
            l2, lw = res.hand_landmarks[0], res.hand_world_landmarks[0]
            hd = f'{res.handedness[0][0].category_name} {res.handedness[0][0].score:.2f}'
            pts = [(int(x0 + l.x * tile.shape[1]), int(y0 + l.y * tile.shape[0])) for l in l2]
            for a, b in HAND_LINKS:
                cv2.line(canvas, pts[a], pts[b], (0, 200, 255), 1, cv2.LINE_AA)
            for i, pt in enumerate(pts):
                cv2.circle(canvas, pt, 3 if i in TIP else 2,
                           (80, 220, 80) if i in TIP else (0, 90, 255) if i == 0 else (255, 255, 255), -1)
            g = Geometry.compute(l2, lw, 0.15)
            exp = EXPECT.get(cls)
            good = (exp is None) or (g['n_ext'] == exp)
            ok_msg = f'伸{g["n_ext"]}' + ('' if exp is None else f'/期望{exp}' + ('✓' if good else '✗'))
            col = (80, 220, 80) if good else (60, 60, 240)
            texts.append((f'{math.degrees(g["theta"]):+.0f}° phi{math.degrees(g["phi"]):+.0f} psi{math.degrees(g["psi"]):+.0f}',
                          (2, T + 4), (210, 210, 210), 12))
            st = stats.setdefault(cls, {'n': 0, 'det': 0, 'ext': {}, 'handed': {}, 'good': 0,
                                        'theta': [], 'phi': []})
            st['det'] += 1
            st['ext'][g['n_ext']] = st['ext'].get(g['n_ext'], 0) + 1
            st['handed'][hd.split()[0]] = st['handed'].get(hd.split()[0], 0) + 1
            st['good'] += int(good)
            st['theta'].append(math.degrees(g['theta']))
            st['phi'].append(math.degrees(g['phi']))
        st = stats.setdefault(cls, {'n': 0, 'det': 0, 'ext': {}, 'handed': {}, 'good': 0,
                                    'theta': [], 'phi': []})
        st['n'] += 1
        if not res.hand_landmarks:
            st['handed']['-'] = st['handed'].get('-', 0) + 1
        texts.append((f'{cls}  {ok_msg}  {hd}', (2, T + 20), col, 13))
        tiles.append(painter.flush(canvas, texts))

    nrow = (len(tiles) + per_row - 1) // per_row
    while len(tiles) < nrow * per_row:
        tiles.append(np.zeros_like(tiles[0]))
    grid = np.vstack([np.hstack(tiles[r * per_row:(r + 1) * per_row]) for r in range(nrow)])

    out = Path(args.out) if args.out else DEFAULT_OUT / f'sheet_{time.strftime("%Y%m%d_%H%M%S")}.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), grid)

    summ = {}
    for cls, st in stats.items():
        summ[cls] = {'n': st['n'], 'detected': st['det'],
                     'ext_hist': {int(k): int(v) for k, v in sorted(st['ext'].items())},
                     'handedness': st['handed'],
                     'match_expect': st['good'], 'expect': EXPECT.get(cls),
                     'theta_median_deg': round(float(np.median(st['theta'])), 1) if st['theta'] else None,
                     'phi_median_deg': round(float(np.median(st['phi'])), 1) if st['phi'] else None}
    jpath = out.with_suffix('.json')
    jpath.write_text(json.dumps(summ, ensure_ascii=False, indent=1), encoding='utf-8')

    if not args.quiet:
        print(f'[完成] {len(items)} 张 / {time.time() - t0:.1f}s -> {out}')
        for cls, s in summ.items():
            print(f'  {cls:11s} 检出 {s["detected"]}/{s["n"]}  伸指数分布 {s["ext_hist"]}  '
                  f'期望 {s["expect"]}  符合 {s["match_expect"]}  手性 {s["handedness"]}')
        print(f'  明细 -> {jpath}')
    close_quietly(lm)
    return 0


if __name__ == '__main__':
    _code = main()
    sys.stdout.flush()
    os._exit(_code if isinstance(_code, int) else 0)
