"""检测网「误触发 / 拒识」评测：一次遍历 val 集，同时算正样本指标与负样本误触发率。

为什么需要它：官方 eval.py 只看正样本(中心误差+抖动)，**看不见"没有手却报警"**。
2026-10-14 实测发现冻结模型在主人实验室空镜(L1)上 conf 中位 0.617、最高 0.926，
即"没人做手势却以为有手"。本脚本把这一类错误量出来。

用法（工程根）:
  cd model && HAGRID_ROOT=data/cache256 python ../results/w1_detect_neg/eval_negatives.py \
      --weights ../results/w1_detect_neg/_neg.pt --tag neg
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'model'))
sys.path.insert(0, str(ROOT / 'model' / 'detect'))
from net import DetectNet                     # noqa: E402
from common.qfake import calibrate_from_loader  # noqa: E402
from dataset import DetectDataset             # noqa: E402

THRESH = (0.2, 0.3, 0.5, 0.7, 0.9)


def summary(confs, errs=None):
    c = np.array(confs, dtype=np.float64)
    out = {'n': int(c.size)}
    if c.size:
        out['conf_median'] = round(float(np.median(c)), 4)
        out['conf_max'] = round(float(c.max()), 4)
        for t in THRESH:
            out[f'fire_rate@{t}'] = round(float((c > t).mean()), 4)
    if errs is not None and len(errs):
        e = np.array(errs)
        out.update({'err_median_px': round(float(np.median(e)), 3),
                    'err_p90_px': round(float(np.percentile(e, 90)), 3),
                    'err_frac_le_8px': round(float((e <= 8).mean()), 4)})
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--weights', type=Path, required=True)
    ap.add_argument('--tag', default='run')
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--out', type=Path, default=None)
    ap.add_argument('--int8', choices=['none', 'sym', 'affine'], default='none')
    ap.add_argument('--int8-report', type=Path, default=None)
    args = ap.parse_args()

    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = DetectNet().to(device).eval()
    net.load_state_dict(torch.load(args.weights, map_location=device))
    ds = DetectDataset('val', augment=False)
    dl = torch.utils.data.DataLoader(ds, batch_size=args.batch, shuffle=False,
                                     num_workers=args.workers)
    if args.int8 != 'none':
        net, qrep = calibrate_from_loader(net, dl, mode=args.int8,
                                          also_quantize=('uv_head.2',), device=device)
        if args.int8_report:
            args.int8_report.parent.mkdir(parents=True, exist_ok=True)
            args.int8_report.write_text(json.dumps(qrep, indent=1), encoding='utf-8')
    n_pos, n_hag = len(ds.pos), len(ds.neg)
    rec_clip = [p.parent.name for p in ds.neg_extra]

    pos_conf, pos_err = [], []          # 被接受的(conf>0.5)正样本
    pos_conf_all, pos_err_all = [], []  # 全部正样本
    hag_conf, rec_conf = [], {}
    off = 0
    with torch.no_grad():
        for x, y in dl:
            p = net.predict(x.to(device)).cpu().numpy()
            y = y.numpy()
            for i in range(len(y)):
                gi, conf = off + i, float(p[i, 0])
                if gi < n_pos:                                  # 正样本
                    e = float(np.hypot(p[i, 1] - y[i, 1], p[i, 2] - y[i, 2]) * 144)
                    pos_conf_all.append(conf); pos_err_all.append(e)
                    if conf > 0.5:                              # 与官方 eval.py 同口径
                        pos_conf.append(conf); pos_err.append(e)
                else:                                           # 负样本
                    k = gi - n_pos
                    if k < n_hag:
                        hag_conf.append(conf)
                    else:
                        rec_conf.setdefault(rec_clip[k - n_hag], []).append(conf)
            off += len(y)

    pos = summary(pos_conf, pos_err)
    pos['n_pos_total'] = int(n_pos)
    pos['recall@0.5'] = round(len(pos_conf) / max(1, n_pos), 4)
    pos['positives_all_samples'] = summary(pos_conf_all, pos_err_all)
    rec = {clip: summary(c) for clip, c in sorted(rec_conf.items())}
    rec['ALL'] = summary([c for cc in rec_conf.values() for c in cc])
    all_neg = hag_conf + [c for cc in rec_conf.values() for c in cc]
    try:
        wstr = str(args.weights.relative_to(ROOT))
    except ValueError:                     # 绝对/相对混用时退回原字符串
        wstr = str(args.weights)
    res = {
        'tag': args.tag, 'weights': wstr, 'int8': args.int8,
        'positives_val': pos,
        'negatives': {
            'hagrid_no_gesture': summary(hag_conf),
            'recorded_in_domain': rec,
            'all_negatives': summary(all_neg),
        },
        'note': 'fire_rate = 该源 conf 超过阈值的比例（负样本上即误触发率）',
    }
    out = args.out or (ROOT / 'results' / 'w1_detect_neg' / f'neg_eval_{args.tag}.json')
    out.write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(res, indent=1, ensure_ascii=False))
    print('已写入', out)


if __name__ == '__main__':
    main()
