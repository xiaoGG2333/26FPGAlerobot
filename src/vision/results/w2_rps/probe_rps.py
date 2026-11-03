"""在 webcam 域数据(RPS: paper/rock/scissors)上测我们冻结模型的真实表现。只读，不改任何模型。

为什么要做: 主人的实时演示"不稳"，离线 HaGRID 指标却达标 —— 差异只能来自域(域漂移)。
本脚本把域漂移量化成三个数:
  A 检测器: 在这批 webcam 图上的置信度分布 + 触发率  (对比 HaGRID val 召回 95.7%)
  B 分类器: 用检测器预测中心造部署几何 ROI -> 三类准确率/混淆
  C 分类器: 用画面正中造同一 ROI(对照组) -> 分离"检测中心误差"与"分类器本身"的贡献

标签映射: paper=张开(palm,1)  rock=闭合(fist,0)  scissors=其他(other,2)
用法: python results/w2_rps/probe_rps.py            # 默认跑 test 集 372 张
      python results/w2_rps/probe_rps.py --split train --limit 500
"""
import argparse
import json
import sys
import time
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'model'))
from common.transforms import normalize                 # noqa: E402
from detect.net import DetectNet                        # noqa: E402
from classify.net import ClassifyNet                    # noqa: E402
from classify.dataset import crop_roi                   # noqa: E402

RPS_ROOT = ROOT / 'model' / 'data' / 'external' / 'rps'
CLS_OF = {'paper': 1, 'rock': 0, 'scissors': 2}          # -> 张开 / 闭合 / 其他
NAME = {0: '闭合(fist)', 1: '张开(palm)', 2: '其他(other)'}
DET_SIZE, CLS_SIZE = 144, 80


def det_input(rgb, size=DET_SIZE):
    h, w = rgb.shape[:2]
    k = size / min(h, w)
    nh, nw = int(round(h * k)), int(round(w * k))
    img = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_AREA)
    y0, x0 = max(0, (nh - size) // 2), max(0, (nw - size) // 2)
    return img[y0:y0 + size, x0:x0 + size]


def t(x):
    return torch.from_numpy(normalize(x).transpose(2, 0, 1)).unsqueeze(0).float()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--split', default='test', choices=['test', 'train'])
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--roi', type=float, default=0.35)
    ap.add_argument('--out', default=str(ROOT / 'results' / 'w2_rps'))
    args = ap.parse_args()

    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    det = DetectNet('softargmax').to(dev).eval()
    det.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'detect_fp32.pt', map_location=dev))
    cls = ClassifyNet().to(dev).eval()
    cls.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'classify_fp32.pt', map_location=dev))

    files = []
    for c in ('paper', 'rock', 'scissors'):
        fs = sorted((RPS_ROOT / args.split / c).glob('*.png'))
        if args.limit:
            fs = fs[:args.limit]
        files += [(p, c) for p in fs]
    print(f'[数据] {args.split} 集 {len(files)} 张  ({Counter(c for _, c in files)})')
    print(f'[标签] paper->张开(1)  rock->闭合(0)  scissors->其他(2)\n')

    conf_all, rows = [], []
    accB, accC = Counter(), Counter()
    cmB, cmC = defaultdict(Counter), defaultdict(Counter)
    dist_ctr = []
    t0 = time.time()
    for p, c in files:
        bgr = cv2.imread(str(p))
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        s = min(h, w)
        with torch.no_grad():
            pr = det.predict(t(det_input(rgb)).to(dev))[0].cpu().numpy()
        conf, u, v = float(pr[0]), float(np.clip(pr[1], 0, 1)), float(np.clip(pr[2], 0, 1))
        conf_all.append(conf)
        px, py = (w - s) / 2 + u * s, (h - s) / 2 + v * s
        dist_ctr.append(float(np.hypot(px - w / 2, py - h / 2) / s * 144))
        gt = CLS_OF[c]

        def classify(cx, cy):
            side = args.roi * s
            bbox = ((cx - side / 2) / w, (cy - side / 2) / h, side / w, side / h)
            roi = crop_roi(rgb, bbox, 1.5, None, CLS_SIZE, False)
            with torch.no_grad():
                lg = cls(t(roi).to(dev))[0]
                return int(torch.softmax(lg, 0).argmax())

        pB = classify(px, py)              # 检测器预测中心
        pC = classify(w / 2, h / 2)        # 画面正中(对照)
        accB[gt] += int(pB == gt)
        accC[gt] += int(pC == gt)
        cmB[gt][pB] += 1
        cmC[gt][pC] += 1
        rows.append({'img': p.name, 'gt': c, 'conf': round(conf, 4),
                     'uv': [round(u, 4), round(v, 4)], 'pred_center_off_px': round(dist_ctr[-1], 2),
                     'cls_from_det': pB, 'cls_from_center': pC})

    n = len(files)
    conf = np.array(conf_all)
    rates = {th: float((conf >= th).mean()) for th in (0.2, 0.3, 0.5, 0.7, 0.9)}
    print(f'[A 检测器] conf 中位 {np.median(conf):.3f}  p10 {np.percentile(conf,10):.3f}  '
          f'p90 {np.percentile(conf,90):.3f}')
    print('           触发率 ' + '  '.join(f'@{k}:{v*100:.1f}%' for k, v in rates.items()))
    print(f'           预测中心离画面中心 中位 {np.median(dist_ctr):.1f}px(p90 {np.percentile(dist_ctr,90):.1f})')
    print(f'           对照 HaGRID val 正样本: 召回@0.5 = 95.7%, conf 中位 0.99+')
    tot = sum(accB.values())
    print(f'\n[B 分类器 | 用检测器中心] 准确率 {sum(accB.values())/n*100:.1f}%')
    for k in (0, 1, 2):
        sup = sum(cmB[k].values())
        if sup:
            print(f'   {NAME[k]:12s} 样本{sup:4d}  对{accB[k]:4d} ({accB[k]/sup*100:5.1f}%)  误判-> ' +
                  ' '.join(f'{NAME[j]}:{v}' for j, v in sorted(cmB[k].items()) if j != k))
    print(f'\n[C 分类器 | 用画面正中(对照)] 准确率 {sum(accC.values())/n*100:.1f}%')
    for k in (0, 1, 2):
        sup = sum(cmC[k].values())
        if sup:
            print(f'   {NAME[k]:12s} 样本{sup:4d}  对{accC[k]:4d} ({accC[k]/sup*100:5.1f}%)  误判-> ' +
                  ' '.join(f'{NAME[j]}:{v}' for j, v in sorted(cmC[k].items()) if j != k))
    other_motion = (cmB[2][0] + cmB[2][1]) / max(1, sum(cmB[2].values()))
    print(f'\n[关键安全指标] 其他手势误动率(剪刀被判成张开/闭合) = {other_motion*100:.1f}%  '
          f'(离线 HaGRID 上 7.11%，门槛 5%)')

    out = {'split': args.split, 'n': n, 'roi_frac': args.roi,
           'detect': {'conf_median': float(np.median(conf)), 'conf_p10': float(np.percentile(conf, 10)),
                      'fire_rate': rates, 'center_off_px_median': float(np.median(dist_ctr))},
           'cls_from_det': {'acc': sum(accB.values()) / n,
                            'per_class': {int(k): {'n': sum(cmB[k].values()), 'correct': accB[k],
                                                   'cm': {int(j): v for j, v in cmB[k].items()}} for k in cmB}},
           'cls_from_center': {'acc': sum(accC.values()) / n},
           'other_motion_rate': other_motion, 'rows': rows}
    Path(args.out).mkdir(parents=True, exist_ok=True)
    (Path(args.out) / f'probe_rps_{args.split}.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'\n[耗时] {time.time()-t0:.1f}s  ->  {args.out}/probe_rps_{args.split}.json')


if __name__ == '__main__':
    main()
