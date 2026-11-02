# -*- coding: utf-8 -*-
"""T3：把打标产物建成训练缓存（256x256 uint8 .npy，mmap 友好）+ 时间块状 70/15/15 划分。

用法（hand-pi 或 train-pi 均可，只依赖 numpy/opencv）：
  python -u tools/build_cache.py --video results/w3_pose/xxx.mp4 \
         --labels results/stage1/xxx_labels.npz --out results/stage1/cache_xxx
样本 = valid 帧（presence=1）+ discard 原因 1 的无手帧（presence=0 负样本）。
划分按时间块状（不打乱）：单视频划分只验"可训练性"，泛化留给 M2 多视频划分（计划书 §4.4）。
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--labels', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--size', type=int, default=256)
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    z = np.load(args.labels)
    pts, valid, discard = z['pts'], z['valid'], z['discard']
    n = len(valid)

    pos_idx = [i for i in range(n) if valid[i]]
    neg_idx = [i for i in range(n) if discard[i] == 1]      # 无手帧当 presence 负样本
    print(f'[划分源] valid={len(pos_idx)}  无手负样本={len(neg_idx)}', flush=True)
    if len(neg_idx) < 100:
        print('[注] 无手负样本 <100：presence 头只训不评（交接文档 T3 约定）', flush=True)

    # 时间块状划分（在 valid 帧的时序上切 70/15/15）
    n_pos = len(pos_idx)
    a, b = int(n_pos * 0.70), int(n_pos * 0.85)
    split = {'train': pos_idx[:a], 'val': pos_idx[a:b], 'test': pos_idx[b:],
             'neg': neg_idx}

    want = sorted(set(pos_idx) | set(neg_idx))
    cap = cv2.VideoCapture(str(args.video))
    imgs = np.zeros((len(want), args.size, args.size, 3), np.uint8)
    got = {}
    want_set = set(want)
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx in want_set:
            got[idx] = cv2.resize(frame, (args.size, args.size),
                                  interpolation=cv2.INTER_AREA)
            if len(got) == len(want_set):
                break
        idx += 1
    cap.release()
    missing = [i for i in want if i not in got]
    if missing:
        print(f'[警告] {len(missing)} 帧解码失败，剔除: {missing[:10]}', flush=True)
        want = [i for i in want if i in got]
        for k in ('train', 'val', 'test', 'neg'):
            split[k] = [i for i in split[k] if i in got]
    for k, i in enumerate(want):
        imgs[k] = got[i]

    remap = {old: new for new, old in enumerate(want)}
    split_remapped = {k: [remap[i] for i in v] for k, v in split.items()}

    samples = np.array(want)
    presence = np.array([1 if valid[i] else 0 for i in want], np.float32)
    hand_code = np.load(args.labels)['hand_code'][samples]      # 0无/1Left/2Right（教师原始）
    scale = np.load(args.labels)['hand_len'][samples].astype(np.float32)

    np.save(out / 'imgs.npy', imgs)
    np.save(out / 'pts.npy', pts[samples].astype(np.float32))
    np.save(out / 'presence.npy', presence)
    np.save(out / 'hand.npy', hand_code.astype(np.int64))
    np.save(out / 'scale.npy', scale)
    (out / 'split.json').write_text(json.dumps(split_remapped), encoding='utf-8')
    meta = {'video': str(args.video), 'labels': str(args.labels), 'size': args.size,
            'n_samples': len(want),
            'counts': {k: len(v) for k, v in split_remapped.items()},
            'temporal': True,
            'note': '时间块状划分；resize 直接 squash 到 256x256（归一化坐标与宽高比无关）'}
    (out / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1),
                                   encoding='utf-8')
    print(f'[完成] {out}  样本 {len(want)}  '
          f'train/val/test/neg = {len(split_remapped["train"])}/{len(split_remapped["val"])}'
          f'/{len(split_remapped["test"])}/{len(split_remapped["neg"])}', flush=True)


if __name__ == '__main__':
    main()
