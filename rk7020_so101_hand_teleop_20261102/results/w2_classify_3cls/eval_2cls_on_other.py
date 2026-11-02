"""对比基线: 旧**二分类**(fist/palm)模型在"其它手势"样本上的误动作率。

旧二分类模型没有"其它"类，任何输入都会输出 fist 或 palm → 在其它手势上理论误动作率 100%。
本脚本实测出来，作为三分类模型的对照证据。

用法（工程根）: cd model/classify && HAGRID_ROOT=data/cache256 python \
    ../../results/w2_classify_3cls/eval_2cls_on_other.py --weights <2类权重>
"""
import argparse
import json
from collections import Counter
from pathlib import Path

import torch

import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'model'))
sys.path.insert(0, str(ROOT / 'model' / 'classify'))
from net import ClassifyNet            # noqa: E402
from dataset import ClassifyDataset, CLASSES   # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--weights', type=Path, required=True)
    ap.add_argument('--batch', type=int, default=128)
    ap.add_argument('--n-classes', type=int, default=2, dest='n_classes')
    ap.add_argument('--out', type=Path, default=ROOT / 'results' / 'w2_classify_3cls' / 'eval_2cls_on_other.json')
    args = ap.parse_args()
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = ClassifyNet(args.n_classes).to(device).eval()
    net.load_state_dict(torch.load(args.weights, map_location=device))
    ds = ClassifyDataset('val', augment=False)
    # 只取二分类能判的样本: fist/palm 看是否判对, other 看是否被误判成 fist/palm(误动作)
    by_src = {}
    correct = total = 0
    other_motion = other_n = 0
    with torch.no_grad():
        for i in range(0, len(ds.items), args.batch):
            chunk = list(range(i, min(i + args.batch, len(ds.items))))
            xs, ys = zip(*(ds[j] for j in chunk))
            x = torch.from_numpy(__import__('numpy').stack(xs)).to(device)
            pred = net(x).argmax(1).cpu().numpy()
            for k, j in enumerate(chunk):
                it, gt = ds.items[j], int(ys[k])
                src = by_src.setdefault(it['source'], {'n': 0, 'to_fist': 0, 'to_palm': 0,
                                                       'correct': 0})
                src['n'] += 1
                src['to_fist'] += int(pred[k] == 0)
                src['to_palm'] += int(pred[k] == 1)
                src['correct'] += int(pred[k] == gt) if it['cls'] != 2 else 0
                if it['cls'] == 2:
                    other_n += 1
                    other_motion += int(pred[k] in (0, 1))
                else:
                    total += 1
                    correct += int(pred[k] == gt)
    for v in by_src.values():
        v['motion_rate'] = round((v['to_fist'] + v['to_palm']) / max(1, v['n']), 4)
    res = {
        'weights': str(args.weights),
        'model': f'{args.n_classes}类(旧: fist/palm)',
        'binary_acc_on_fist_palm_val': round(correct / max(1, total), 4),
        'binary_val_n': total,
        'other_motion_rate': round(other_motion / max(1, other_n), 4),
        'other_n': other_n,
        'by_source': by_src,
        'note': '旧模型无"其它"类 → 其它手势上只能输出 fist/palm, 即夹爪一定会动',
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(res, indent=1, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(res, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
