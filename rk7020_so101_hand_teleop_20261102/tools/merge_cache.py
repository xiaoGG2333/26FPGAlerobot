# -*- coding: utf-8 -*-
"""把多个自采缓存合并成一个（train-pi 环境，纯 numpy 拷贝，不跑 MediaPipe）。

用途：v1（第一支录像，含握拳/摇杆握持）+ v2（第二支，手由近及远 + 空手负样本）
合并后一起训，避免"新数据把旧姿态冲掉"（self_run2 就是握拳被冲没了）。

约定：
- 每个源缓存的 imgs/pts/presence/hand/scale/roi 按顺序拼接
- 负样本统一放到末尾，并写进 split['neg']
- --train-all 时不留 test，val 只取 n_val 张均匀分布的**正样本**做监控信号
- 额外写 source.npy（每个样本来自哪个缓存），方便以后按来源加权/排查

用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/merge_cache.py \
      --caches results/stage1/cache_self_v1 results/stage1/cache_self_v2 \
      --out results/stage1/cache_self_v3 --n-val 128
"""
import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
KEYS = ('imgs', 'pts', 'presence', 'hand', 'scale', 'roi')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--caches', nargs='+', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--n-val', type=int, default=128)
    ap.add_argument('--train-all', action='store_true', default=True)
    ap.add_argument('--splits', default='train,val,test,neg',
                    help='从每个源缓存取哪些 split（默认全取；只想取训练部分就传 train,neg）')
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    parts = {k: [] for k in KEYS}
    src, counts = [], []
    for ci, c in enumerate(args.caches):
        c = Path(c)
        m = json.loads((c / 'meta.json').read_text())
        spl = json.loads((c / 'split.json').read_text())
        used = spl['train'] + spl.get('val', []) + spl.get('test', []) + spl.get('neg', [])
        keep = [k.strip() for k in args.splits.split(',') if k.strip()]
        used = [i for k in keep for i in spl.get(k, [])]
        pos = [i for i in used if i not in set(spl.get('neg', []))]
        neg = list(spl.get('neg', []))
        order = pos + neg                      # 正样本在前、负样本在后
        for k in KEYS:
            a = np.load(c / f'{k}.npy', mmap_mode='r')
            parts[k].append(np.array(a[order]))
        src.append(np.full(len(order), ci, np.int64))
        counts.append({'cache': str(c), 'pos': len(pos), 'neg': len(neg),
                       'hand_pct': m.get('hand_pct')})
        print(f'[读入] {c}  正 {len(pos)}  负 {len(neg)}')

    data = {}
    for k in KEYS:
        data[k] = np.concatenate(parts[k]) if len(parts[k]) > 1 else parts[k][0]
    source = np.concatenate(src)
    pres = data['presence']
    pos_idx = np.where(pres > 0.5)[0]
    neg_idx = np.where(pres <= 0.5)[0]
    n = len(pres)

    step = max(1, len(pos_idx) // max(1, args.n_val))
    val = sorted(pos_idx[::step][:args.n_val].tolist())
    split = {'train': sorted(pos_idx.tolist()) + sorted(neg_idx.tolist()),
             'val': val, 'test': [], 'neg': sorted(neg_idx.tolist())}

    for k in KEYS:
        np.save(out / f'{k}.npy', data[k])
    np.save(out / 'source.npy', source)
    (out / 'split.json').write_text(json.dumps(split), encoding='utf-8')

    s_pos = data['scale'][pres > 0.5]
    meta = {'sources': counts, 'n_samples': int(n), 'size': 256,
            'counts': {'train': len(split['train']), 'val': len(val),
                       'test': 0, 'neg': len(neg_idx)},
            'pos_scale_pct': [round(float(x), 4) for x in
                              np.percentile(s_pos, [5, 25, 50, 75, 95])],
            'split_mode': 'merged_all_train'}
    (out / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f"[合并] 总 {n}（正 {len(pos_idx)} / 负 {len(neg_idx)}）"
          f"  正样本 hand/ROI 中位数 {np.median(s_pos):.4f}")
    print(f'[划分] {meta["counts"]}')
    print(f'[输出] {out}')


if __name__ == '__main__':
    main()
