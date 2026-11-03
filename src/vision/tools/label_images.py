# -*- coding: utf-8 -*-
"""T2c：图像数据集蒸馏打标（HaGRID 等）+ 尺度增广（远近覆盖）+ 随机划分缓存。

与 label_auto（视频版）的差异：
- 图像 i.i.d. → 随机 8:1:1 划分；**先按图像划分再增广**，同源增广样本不跨 split（防泄漏）
- 尺度增广：裁剪窗缩放 s，hp_new = hp_old * s；s 由目标 hp ~ loguniform(0.06, 0.34) 反解
  （部署带 0.12~0.22 在区间中段；HaGRID 原始中位仅 0.085，近端靠增广补）
- 裁剪窗保持原图宽高比；窗越界用 copyMakeBorder(REPLICATE) 补
- 负样本：窗中心偏到手完全在窗外（presence=0），比例 --neg-ratio
产物布局与 build_cache 一致：imgs/pts/presence/hand/scale .npy + split.json + meta.json
另出 distill_check.png（按 hp 分层：近/带内/远 各 12 张增广样本）给主人过目。

用法（hand-pi）：
  D:/Anaconda3/envs/hand-pi/python.exe -u tools/label_images.py \
      --roots model/data/hagrid_subset --out results/stage1/cache_hagrid --aug-per 2
"""
import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sw'))
import pose_map  # noqa: E402

HP_KEEP = (0.04, 0.45)          # 教师检出后的手长保留窗（过小的标注噪声大）
HP_TARGET = (0.06, 0.34)        # 增广目标手长区间（log-uniform）
S_CLAMP = (0.3, 4.0)


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument('--roots', nargs='+', default=['model/data/hagrid_subset'])
    ap.add_argument('--out', default='results/stage1/cache_hagrid')
    ap.add_argument('--size', type=int, default=256)
    ap.add_argument('--aug-per', type=int, default=2)
    ap.add_argument('--neg-ratio', type=float, default=0.10)
    ap.add_argument('--min-score', type=float, default=0.8)
    ap.add_argument('--seed', type=int, default=11)
    ap.add_argument('--max-images', type=int, default=0)
    return ap.parse_args()


def window_params(W, H, c_px, hp, rng, kind):
    """返回 (s, cx, cy)：kind='base' 全图；'aug' 按目标 hp 反解 s；'neg' 手在窗外。"""
    if kind == 'base':
        return 1.0, W / 2.0, H / 2.0
    if kind == 'aug':
        target = math.exp(rng.uniform(math.log(HP_TARGET[0]), math.log(HP_TARGET[1])))
        s = min(S_CLAMP[1], max(S_CLAMP[0], target / max(hp, 1e-6)))
        jx = rng.uniform(-0.10, 0.10) * (W / s)
        jy = rng.uniform(-0.10, 0.10) * (H / s)
        return s, c_px[0] + jx, c_px[1] + jy
    # neg：窗完全不含手——窗中心沿随机方向推离手中心 1.2 个窗对角
    s = rng.uniform(0.8, 2.0)
    wW, wH = W / s, H / s
    ang = rng.uniform(0, 2 * math.pi)
    d = 0.75 * math.hypot(wW, wH)
    return s, c_px[0] + d * math.cos(ang), c_px[1] + d * math.sin(ang)


def render(img, W, H, s, cx, cy, size):
    wW, wH = W / s, H / s
    x0, y0 = cx - wW / 2.0, cy - wH / 2.0
    pl, pr = max(0, -int(math.floor(x0))), max(0, int(math.ceil(x0 + wW)) - W)
    pt, pb = max(0, -int(math.floor(y0))), max(0, int(math.ceil(y0 + wH)) - H)
    if pl or pr or pt or pb:
        img = cv2.copyMakeBorder(img, pt, pb, pl, pr, cv2.BORDER_REPLICATE)
        x0, y0 = x0 + pl, y0 + pt
    crop = img[int(round(y0)):int(round(y0)) + int(round(wH)),
               int(round(x0)):int(round(x0)) + int(round(wW))]
    if crop.size == 0:
        return None, x0, y0, wW, wH
    return cv2.resize(crop, (size, size), interpolation=cv2.INTER_AREA), x0, y0, wW, wH


def main():
    args = parse()
    rng = random.Random(args.seed)
    np.random.seed(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    size = args.size

    files = []
    for r in args.roots:
        rp = ROOT / r if not Path(r).is_absolute() else Path(r)
        files += sorted(rp.rglob('*.jpg')) + sorted(rp.rglob('*.png'))
    rng.shuffle(files)
    if args.max_images:
        files = files[:args.max_images]
    n = len(files)
    a, b = int(n * 0.8), int(n * 0.9)
    split_of_file = ['train'] * a + ['val'] * (b - a) + ['test'] * (n - b)
    print(f'[输入] {n} 张图像  train/val/test = {a}/{b-a}/{n-b}', flush=True)

    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions
    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(
            ROOT / 'model/data/external/mediapipe/hand_landmarker.task')),
        running_mode=vision.RunningMode.IMAGE, num_hands=2)
    lm = vision.HandLandmarker.create_from_options(opts)

    # ---- pass 1：教师统计 + 样本规划 ----
    metas, plan = [], []          # plan: (meta_idx or -1, kind, s, cx, cy)
    cnt = {'no_hand': 0, 'multi': 0, 'low_score': 0, 'hp_out': 0, 'keep': 0}
    t0 = time.time()
    for i, f in enumerate(files):
        img = cv2.imread(str(f))
        if img is None:
            cnt['no_hand'] += 1
            continue
        H, W = img.shape[:2]
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        res = lm.detect(mp.Image(image_format=mp.ImageFormat.SRGB,
                                 data=np.ascontiguousarray(rgb)))
        if len(res.hand_landmarks) == 0:
            cnt['no_hand'] += 1
            continue
        if len(res.hand_landmarks) > 1:
            cnt['multi'] += 1
            continue
        sc = float(res.handedness[0][0].score)
        if sc < args.min_score:
            cnt['low_score'] += 1
            continue
        p = np.array([[q.x, q.y] for q in res.hand_landmarks[0]], np.float32)
        hp = float(np.linalg.norm(p[9] - p[0]))
        if not (HP_KEEP[0] <= hp <= HP_KEEP[1]):
            cnt['hp_out'] += 1
            continue
        cnt['keep'] += 1
        mi = len(metas)
        metas.append({'file': str(f), 'W': W, 'H': H, 'pts': p, 'hp': hp,
                      'hand': {'Left': 1, 'Right': 2}.get(
                          str(res.handedness[0][0].category_name), 0),
                      'split': split_of_file[i]})
        c_px = (float(p[:, 0].mean()) * W, float(p[:, 1].mean()) * H)
        plan.append((mi, 'base', 1.0, W / 2.0, H / 2.0))
        for _ in range(args.aug_per):
            s, cx, cy = window_params(W, H, c_px, hp, rng, 'aug')
            plan.append((mi, 'aug', s, cx, cy))
        if rng.random() < args.neg_ratio:
            s, cx, cy = window_params(W, H, c_px, hp, rng, 'neg')
            plan.append((-1 - mi, 'neg', s, cx, cy))     # 负样本记 -(mi+1)
        if (i + 1) % 1000 == 0:
            print(f'[pass1] {i+1}/{n}  keep={cnt["keep"]}  { (i+1)/(time.time()-t0):.0f} img/s',
                  flush=True)
    print(f'[pass1] 统计 {cnt}  样本计划 {len(plan)} 条', flush=True)
    pose_map.close_quietly(lm)

    # ---- 坐标/标签在规划期即可算出（不需要像素）----
    N = len(plan)
    pts = np.zeros((N, 21, 2), np.float32)
    presence = np.zeros(N, np.float32)
    hand = np.zeros(N, np.int64)
    scale = np.zeros(N, np.float32)
    split_idx = {'train': [], 'val': [], 'test': [], 'neg': []}
    for k, (mi, kind, s, cx, cy) in enumerate(plan):
        if kind == 'neg':
            mi = -mi - 1
            split_idx['neg'].append(k)
            continue
        m = metas[mi]
        W, H = m['W'], m['H']
        wW, wH = W / s, H / s
        x0, y0 = cx - wW / 2.0, cy - wH / 2.0
        pn = (m['pts'] * np.array([W, H], np.float32) - np.array([x0, y0], np.float32)) \
            / np.array([wW, wH], np.float32)
        pts[k] = pn
        presence[k] = 1.0
        hand[k] = m['hand']
        scale[k] = m['hp'] * s
        split_idx[m['split']].append(k)

    # ---- pass 2：渲染像素进 memmap ----
    imgs = np.lib.format.open_memmap(out / 'imgs.npy', mode='w+', dtype=np.uint8,
                                     shape=(N, size, size, 3))
    t0 = time.time()
    cache_img = {}
    for k, (mi, kind, s, cx, cy) in enumerate(plan):
        src = metas[mi if mi >= 0 else -mi - 1]
        fp = src['file']
        if fp not in cache_img:
            cache_img[fp] = cv2.imread(fp)
            if len(cache_img) > 64:
                cache_img.pop(next(iter(cache_img)))
        r = render(cache_img[fp], src['W'], src['H'], s, cx, cy, size)
        if r[0] is None:
            presence[k] = 0.0
            if k in split_idx[src['split']]:
                split_idx[src['split']].remove(k)
            continue
        imgs[k] = r[0]
        if (k + 1) % 2000 == 0:
            print(f'[pass2] {k+1}/{N}  {(k+1)/(time.time()-t0):.0f} samp/s', flush=True)
    imgs.flush()

    np.save(out / 'pts.npy', pts)
    np.save(out / 'presence.npy', presence)
    np.save(out / 'hand.npy', hand)
    np.save(out / 'scale.npy', scale)
    (out / 'split.json').write_text(json.dumps(split_idx), encoding='utf-8')
    pos_scale = scale[presence > 0]
    meta = {'roots': args.roots, 'size': size, 'n_samples': N,
            'counts': {k: len(v) for k, v in split_idx.items()},
            'pass1_stats': cnt, 'aug_per': args.aug_per, 'neg_ratio': args.neg_ratio,
            'hp_after_aug_percentiles': [round(float(q), 4) for q in
                                          np.percentile(pos_scale, [5, 25, 50, 75, 95])],
            'hp_in_deploy_band': round(float(((pos_scale >= 0.12) & (pos_scale <= 0.22)).mean()), 3),
            'temporal': False,
            'seed': args.seed, 'created': time.strftime('%Y-%m-%d %H:%M:%S')}
    (out / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1),
                                   encoding='utf-8')
    print('[统计] ' + json.dumps({k: len(v) for k, v in split_idx.items()}))
    print(f"[手长] 增广后分位 {meta['hp_after_aug_percentiles']}  "
          f"部署带内占比 {meta['hp_in_deploy_band']}")

    # ---- 证据图：近/带内/远 各 12 张增广样本 ----
    pos_k = [k for k in range(N) if presence[k] > 0 and plan[k][1] == 'aug']
    by_hp = sorted(pos_k, key=lambda k: scale[k])
    picks = by_hp[-12:] + [k for k in by_hp
                           if 0.12 <= scale[k] <= 0.22][:12] + by_hp[:12]
    tiles = []
    for k in picks:
        t = imgs[k].copy()
        px = (pts[k] * size).astype(int)
        inside = ((px >= 0) & (px < size)).all(axis=1)
        for a2, b2 in pose_map.HAND_LINKS:
            if inside[a2] and inside[b2]:
                cv2.line(t, tuple(px[a2]), tuple(px[b2]), (80, 220, 80), 1, cv2.LINE_AA)
        for j in range(21):
            if inside[j]:
                cv2.circle(t, tuple(px[j]), 2, (80, 220, 80), -1, cv2.LINE_AA)
        tag = f'hp={scale[k]:.3f} {plan[k][1]}'
        cv2.putText(t, tag, (3, size - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
        cv2.putText(t, tag, (4, size - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        tiles.append(t)
    cols = 6
    canvas = np.zeros(((len(tiles) + cols - 1) // cols * size, cols * size, 3), np.uint8)
    for k, t in enumerate(tiles):
        canvas[(k // cols) * size:(k // cols + 1) * size,
               (k % cols) * size:(k % cols + 1) * size] = t
    cv2.imwrite(str(ROOT / 'results/stage1/distill_check.png'), canvas)
    print(f'[证据图] results/stage1/distill_check.png（近12/带内12/远12）')
    sys.stdout.flush()
    os_exit = getattr(__import__('os'), '_exit')
    os_exit(0)


if __name__ == '__main__':
    main()
