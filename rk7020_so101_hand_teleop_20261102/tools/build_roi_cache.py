# -*- coding: utf-8 -*-
"""两阶段路线的阶段二数据：教师 bbox 方窗裁切（oracle ROI）-> 256² 缓存。

计划书 §5.2 预登记分支：单阶段全帧 NME 不达标 -> 粗定位 ROI + ROI 内精回归。
阶段一（检测） sanity 阶段用教师框代替（oracle），先回答"裁切后小网能否过门槛"；
阶段一真检测器复用 W1 DetectNet 思路（手掌中心+尺度头），另行立项。

与 label_images 的差异：
- 裁切窗 = 教师 21 点 bbox 的方窗 x margin（margin ~ U(1.3, 2.0) 作增广），原生分辨率裁切再 resize
- 标签为 ROI 内归一化坐标；另存 roi.npy (x0n, y0n, sw, sh) 供评测仿射回全帧（门槛口径不变）
- 无负样本（"手在不在"归阶段一管）；presence 恒 1
用法（hand-pi）：
  D:/Anaconda3/envs/hand-pi/python.exe -u tools/build_roi_cache.py \
      --roots model/data/hagrid_subset --out results/stage1/cache_hagrid_roi
"""
import argparse
import json
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sw'))
import pose_map  # noqa: E402

HP_KEEP = (0.04, 0.45)


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', nargs='+', default=['model/data/hagrid_subset'])
    ap.add_argument('--out', default='results/stage1/cache_hagrid_roi')
    ap.add_argument('--size', type=int, default=256)
    ap.add_argument('--aug-per', type=int, default=1)
    ap.add_argument('--min-score', type=float, default=0.8)
    ap.add_argument('--seed', type=int, default=21)
    return ap.parse_args()


def crop_roi(img, p_norm, margin, rng, size):
    H, W = img.shape[:2]
    p = p_norm * np.array([W, H], np.float32)
    x0, y0 = p.min(axis=0)
    x1, y1 = p.max(axis=0)
    side = max(x1 - x0, y1 - y0) * margin
    side = max(side, 8.0)
    cx, cy = (x0 + x1) / 2 + rng.uniform(-0.05, 0.05) * side, \
             (y0 + y1) / 2 + rng.uniform(-0.05, 0.05) * side
    lx, ly = cx - side / 2, cy - side / 2
    pl, pr = max(0, -int(np.floor(lx))), max(0, int(np.ceil(lx + side)) - W)
    pt, pb = max(0, -int(np.floor(ly))), max(0, int(np.ceil(ly + side)) - H)
    if pl or pr or pt or pb:
        img = cv2.copyMakeBorder(img, pt, pb, pl, pr, cv2.BORDER_REPLICATE)
        lx, ly = lx + pl, ly + pt
    crop = img[int(round(ly)):int(round(ly)) + int(round(side)),
               int(round(lx)):int(round(lx)) + int(round(side))]
    if crop.size == 0:
        return None, None
    crop = cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA)
    pr_norm = (p - np.array([lx, ly], np.float32)) / side
    box = (lx / W, ly / H, side / W, side / H)          # 全帧归一化仿射参数
    return crop, (pr_norm, box)


def main():
    args = parse()
    rng = random.Random(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    files = []
    for r in args.roots:
        rp = ROOT / r
        files += sorted(rp.rglob('*.jpg')) + sorted(rp.rglob('*.png'))
    rng.shuffle(files)
    n = len(files)
    a, b = int(n * 0.8), int(n * 0.9)
    split_of = ['train'] * a + ['val'] * (b - a) + ['test'] * (n - b)

    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions
    lm = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(
            ROOT / 'model/data/external/mediapipe/hand_landmarker.task')),
        running_mode=vision.RunningMode.IMAGE, num_hands=2))

    samples = []          # (file, split, pts_roi, hand, scale_roi, box)
    cnt = {'no_hand': 0, 'multi': 0, 'low_score': 0, 'hp_out': 0, 'keep': 0}
    t0 = time.time()
    for i, f in enumerate(files):
        img = cv2.imread(str(f))
        if img is None:
            cnt['no_hand'] += 1
            continue
        H, W = img.shape[:2]
        det = img
        if max(H, W) > 1280:                      # 关键点为归一化坐标，降分辨率检测不影响标签
            f_sc = 1280.0 / max(H, W)
            det = cv2.resize(img, (int(W * f_sc), int(H * f_sc)))
        rgb = cv2.cvtColor(det, cv2.COLOR_BGR2RGB)
        res = lm.detect(mp.Image(image_format=mp.ImageFormat.SRGB,
                                 data=np.ascontiguousarray(rgb)))
        if not res.hand_landmarks:
            cnt['no_hand'] += 1
            continue
        if len(res.hand_landmarks) > 1:
            cnt['multi'] += 1
            continue
        sc = float(res.handedness[0][0].score)
        p = np.array([[q.x, q.y] for q in res.hand_landmarks[0]], np.float32)
        hp = float(np.linalg.norm(p[9] - p[0]))
        if sc < args.min_score or not (HP_KEEP[0] <= hp <= HP_KEEP[1]):
            cnt['low_score' if sc < args.min_score else 'hp_out'] += 1
            continue
        cnt['keep'] += 1
        hand = {'Left': 1, 'Right': 2}.get(str(res.handedness[0][0].category_name), 0)
        for k in range(args.aug_per + 1):
            margin = 1.5 if k == 0 else rng.uniform(1.3, 2.0)
            r = crop_roi(img, p, margin, rng, args.size)
            if r[0] is None:
                continue
            crop, (pr_norm, box) = r
            samples.append({'file': str(f), 'split': split_of[i], 'img': crop,
                            'pts': pr_norm.astype(np.float32), 'hand': hand,
                            'scale': float(np.linalg.norm(pr_norm[9] - pr_norm[0])),
                            'box': box})
        if (i + 1) % 1000 == 0:
            print(f'[进度] {i+1}/{n} keep={cnt["keep"]} 样本={len(samples)} '
                  f'{(i+1)/(time.time()-t0):.0f} img/s', flush=True)
    pose_map.close_quietly(lm)
    print(f'[统计] {cnt}  样本 {len(samples)}', flush=True)

    N = len(samples)
    imgs = np.lib.format.open_memmap(out / 'imgs.npy', mode='w+', dtype=np.uint8,
                                     shape=(N, args.size, args.size, 3))
    pts = np.zeros((N, 21, 2), np.float32)
    presence = np.ones(N, np.float32)
    hand = np.zeros(N, np.int64)
    scale = np.zeros(N, np.float32)
    roi = np.zeros((N, 4), np.float32)
    split_idx = {'train': [], 'val': [], 'test': [], 'neg': []}
    for k, s in enumerate(samples):
        imgs[k] = s['img']
        pts[k] = s['pts']
        hand[k] = s['hand']
        scale[k] = s['scale']
        roi[k] = s['box']
        split_idx[s['split']].append(k)
    imgs.flush()
    np.save(out / 'pts.npy', pts)
    np.save(out / 'presence.npy', presence)
    np.save(out / 'hand.npy', hand)
    np.save(out / 'scale.npy', scale)
    np.save(out / 'roi.npy', roi)
    (out / 'split.json').write_text(json.dumps(split_idx), encoding='utf-8')
    pos = scale
    meta = {'roots': args.roots, 'size': args.size, 'n_samples': N,
            'counts': {k: len(v) for k, v in split_idx.items()},
            'pass1_stats': cnt, 'stage2': True, 'temporal': False,
            'hp_roi_percentiles': [round(float(q), 4) for q in
                                    np.percentile(pos, [5, 25, 50, 75, 95])],
            'seed': args.seed, 'created': time.strftime('%Y-%m-%d %H:%M:%S')}
    (out / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1),
                                   encoding='utf-8')
    print('[统计] ' + json.dumps(meta['counts']))
    print(f"[ROI 手长] 分位 {meta['hp_roi_percentiles']}")

    pick = [samples[i] for i in random.sample(range(N), 36)]
    tiles = []
    for s in pick:
        t = s['img'].copy()
        px = (s['pts'] * args.size).astype(int)
        for a2, b2 in pose_map.HAND_LINKS:
            cv2.line(t, tuple(px[a2]), tuple(px[b2]), (80, 220, 80), 1, cv2.LINE_AA)
        for j in range(21):
            cv2.circle(t, tuple(px[j]), 2, (80, 220, 80), -1, cv2.LINE_AA)
        tag = f"hp_roi={s['scale']:.2f} {s['split'][:3]}"
        cv2.putText(t, tag, (3, args.size - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
        cv2.putText(t, tag, (4, args.size - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (255, 255, 255), 1)
        tiles.append(t)
    cols = 6
    canvas = np.zeros(((len(tiles) + cols - 1) // cols * args.size,
                       cols * args.size, 3), np.uint8)
    for k, t in enumerate(tiles):
        canvas[(k // cols) * args.size:(k // cols + 1) * args.size,
               (k % cols) * args.size:(k % cols + 1) * args.size] = t
    cv2.imwrite(str(ROOT / 'results/stage1/roi_check.png'), canvas)
    print('[证据图] results/stage1/roi_check.png')
    sys.stdout.flush()
    os_exit = getattr(__import__('os'), '_exit')
    os_exit(0)


if __name__ == '__main__':
    main()
