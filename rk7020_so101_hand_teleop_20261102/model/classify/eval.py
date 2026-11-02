"""三分类评估: 准确率 + 3x3 混淆矩阵 + **误动作率**(other 被判成 fist/palm = 夹爪乱动)。

用法: python eval.py [--weights ../frozen/classify_fp32.pt]

验收口径(建议, 见 results 报告):
  ① overall val acc ≥ 90%(沿用旧门槛)
  ② **误动作率 ≤ 5%**: 标签为 other(竖拇指/OK/剪刀手/自然搭手/无手背景) 的样本
     被判成 fist 或 palm 的比例 —— 这类错误会让夹爪不该动时动
  ③ 张开(palm)/闭合(fist) 各自 recall ≥ 90%
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from net import ClassifyNet, NUM_CLASSES
from dataset import ClassifyDataset, CLASSES

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.qfake import calibrate_from_loader          # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--weights', type=Path, default=ROOT / 'frozen' / 'classify_fp32.pt')
    ap.add_argument('--batch', type=int, default=128)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--int8', choices=['none', 'sym', 'affine'], default='none',
                    help='跑 INT8 假量化推理（先用本 val 集校准）；sym=对称 zp=0, affine=非对称')
    ap.add_argument('--eval-out', type=Path, default=None, help='结果 json 路径（默认 weights 同目录 classify_eval.json）')
    ap.add_argument('--int8-report', type=Path, default=None, help='每层 scale/SNR 表 json 路径')
    args = ap.parse_args()
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = ClassifyNet(NUM_CLASSES).to(device).eval()
    net.load_state_dict(torch.load(args.weights, map_location=device))
    ds = ClassifyDataset('val', augment=False)
    dl = torch.utils.data.DataLoader(ds, batch_size=args.batch, shuffle=False,
                                     num_workers=args.workers, persistent_workers=True)
    if args.int8 != 'none':
        net, rep = calibrate_from_loader(net, dl, mode=args.int8, device=device)
        if args.int8_report:
            args.int8_report.parent.mkdir(parents=True, exist_ok=True)
            args.int8_report.write_text(json.dumps(rep, indent=1), encoding='utf-8')
            print('已写入量化表', args.int8_report)
    n_cls = len(CLASSES)
    cm = np.zeros((n_cls, n_cls), dtype=int)
    rows = []
    off = 0
    with torch.no_grad():
        for x, y in dl:
            pred = net(x.to(device)).argmax(1).cpu().numpy()
            y = y.numpy()
            for i in range(len(y)):
                cm[y[i], pred[i]] += 1
                rows.append((int(y[i]), int(pred[i]), ds.items[off + i]['source']))
            off += len(y)
    assert off == len(ds.items), (off, len(ds.items))

    acc = float(cm.diagonal().sum() / max(1, cm.sum()))
    per_class = {}
    for k, name in enumerate(CLASSES):
        support = int(cm[k].sum())
        tp = int(cm[k, k])
        pred_total = int(cm[:, k].sum())
        per_class[name] = {
            'support': support, 'recall': round(tp / support, 4) if support else None,
            'precision': round(tp / pred_total, 4) if pred_total else None,
        }
    # 误动作率: other → fist/palm
    other_n = sum(1 for g, p, s in rows if g == 2)
    other_move = sum(1 for g, p, s in rows if g == 2 and p != 2)
    by_src = {}
    for g, p, s in rows:
        if g != 2:
            continue
        d = by_src.setdefault(s, {'n': 0, 'to_fist': 0, 'to_palm': 0})
        d['n'] += 1
        d['to_fist'] += (p == 0)
        d['to_palm'] += (p == 1)
    for d in by_src.values():
        d['motion_rate'] = round((d['to_fist'] + d['to_palm']) / max(1, d['n']), 4)

    try:
        wstr = str(args.weights.relative_to(ROOT))
    except ValueError:
        wstr = str(args.weights)
    res = {
        'weights': wstr,
        'classes': list(CLASSES),
        'n': int(cm.sum()), 'val_acc': round(acc, 4),
        'per_class': per_class,
        'confusion': {CLASSES[i]: {CLASSES[j]: int(cm[i, j]) for j in range(n_cls)}
                      for i in range(n_cls)},
        'other_motion_rate': round(other_move / max(1, other_n), 4),
        'other_by_source': by_src,
        'gates': {
            'acc_ge_90': bool(acc >= 0.90),
            'other_motion_le_5pct': bool(other_move / max(1, other_n) <= 0.05),
            'palm_recall_ge_90': bool(per_class['palm']['recall'] >= 0.90),
            'fist_recall_ge_90': bool(per_class['fist']['recall'] >= 0.90),
        },
    }
    out = args.eval_out or (args.weights.parent / 'classify_eval.json')
    if args.int8 != 'none':
        res['int8'] = args.int8
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(res, indent=1, ensure_ascii=False))
    print('已写入', out)


if __name__ == '__main__':
    main()
