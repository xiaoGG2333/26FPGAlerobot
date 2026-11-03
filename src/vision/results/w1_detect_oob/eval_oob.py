"""「手在窗口外」拒识评测 + conf 阈值权衡曲线（只读，口径对齐 detect/eval.py）。

背景（2026-10-15）：val 正样本里有一部分标注中心落在部署中心窗口之外（手没在窗口里），
旧模型给 conf 中位 0.9956、照样输出一个"内部合理点"，误差中位 106px。
本轮训练加入"目标手在窗口外"负样本后，conf 变成可用的信号——但 conf 阈值 0.5 不再是唯一选择：
阈值是**部署时可调的自由参数**，所以这里输出整条权衡曲线，由控制侧挑工作点。

指标定义：
  - 窗口外(OOB)组：拒识率 = conf < t 的比例（越高越好）
  - 窗口内组    ：召回 = conf > t 的比例（越高越好）；误拒 = 1 - 召回
  - 精度只在窗口内被接受的样本上统计（中位/p90/≤8px 占比）——这才是模型真实定位能力
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
from detect.dataset import DetectDataset                    # noqa: E402
from detect.net import DetectNet                            # noqa: E402

THRESHOLDS = (0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 0.9)


@torch.no_grad()
def run(weights, device='cuda', batch=64, workers=2, tag=''):
    net = DetectNet().to(device).eval()
    net.load_state_dict(torch.load(weights, map_location=device))
    dl = torch.utils.data.DataLoader(DetectDataset('val', augment=False),
                                     batch_size=batch, shuffle=False, num_workers=workers)
    conf, err, is_oob, out_d = [], [], [], []
    for x, y in dl:
        pred = net.predict(x.to(device)).cpu().numpy()
        y = y.numpy()
        m = y[:, 0] > 0.5
        if not m.any():
            continue
        u, v, c = y[m, 1], y[m, 2], pred[m, 0]
        conf.append(c)
        err.append(np.hypot(pred[m, 1] - u, pred[m, 2] - v) * 144)
        o = (u < 0.0) | (u > 1.0) | (v < 0.0) | (v > 1.0)
        is_oob.append(o)
        out_d.append(np.hypot(np.maximum(0, u - 1) + np.maximum(0, -u),
                              np.maximum(0, v - 1) + np.maximum(0, -v)) * 144)
    conf, err, is_oob, out_d = (np.concatenate(a) for a in (conf, err, is_oob, out_d))
    in_m, oob_m = ~is_oob, is_oob
    sweep = []
    for t in THRESHOLDS:
        acc_in = err[in_m & (conf > t)]
        sweep.append({
            'thr': t,
            'in_recall': round(float((conf[in_m] > t).mean()), 4),
            'in_false_reject': round(float((conf[in_m] <= t).mean()), 4),
            'oob_reject': round(float((conf[oob_m] <= t).mean()), 4),
            'in_median_px': round(float(np.median(acc_in)), 3) if acc_in.size else None,
            'in_p90_px': round(float(np.percentile(acc_in, 90)), 2) if acc_in.size else None,
            'in_frac_le_8px': round(float((acc_in <= 8).mean()), 4) if acc_in.size else None,
            'n_in_accepted': int(acc_in.size),
        })
    return {'weights': tag or Path(weights).name, 'n_pos': int(conf.size),
            'n_oob': int(oob_m.sum()), 'n_in': int(in_m.sum()),
            'oob_conf_median': round(float(np.median(conf[oob_m])), 4),
            'oob_conf_max': round(float(conf[oob_m].max()), 4),
            'oob_gt_out_px_median': round(float(np.median(out_d[oob_m])), 1),
            'sweep': sweep}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', type=Path, nargs='+', required=True)
    ap.add_argument('--tags', nargs='*', default=None)
    ap.add_argument('--tag', default='oob')
    ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    ap.add_argument('--out-dir', type=Path, default=Path(__file__).resolve().parent)
    args = ap.parse_args()

    tags = args.tags or [w.name for w in args.weights]
    rows = [run(w, args.device, tag=t) for w, t in zip(args.weights, tags)]
    for r in rows:
        print(f"\n[{r['weights']}] 正样本 {r['n_pos']}（窗口外 {r['n_oob']} / 窗口内 {r['n_in']}）"
              f" 窗口外 conf 中位 {r['oob_conf_median']:.4f} 最大 {r['oob_conf_max']:.4f}"
              f" | 真值离窗口中位 {r['oob_gt_out_px_median']}px")
        print(f"  {'阈值':>5s}{'窗口内召回':>11s}{'误拒':>8s}{'窗口外拒识':>11s}"
              f"{'窗口内中位':>11s}{'窗口内p90':>10s}{'≤8px':>8s}{'接受数':>8s}")
        for s in r['sweep']:
            print(f"  {s['thr']:>5.1f}{s['in_recall']:>11.1%}{s['in_false_reject']:>8.1%}"
                  f"{s['oob_reject']:>11.1%}{(s['in_median_px'] or 0):>10.2f}px"
                  f"{(s['in_p90_px'] or 0):>9.1f}px{s['in_frac_le_8px']:>8.1%}{s['n_in_accepted']:>8d}")
    out = args.out_dir / f'eval_{args.tag}.json'
    out.write_text(json.dumps({'rows': rows}, indent=1, ensure_ascii=False), encoding='utf-8')
    print('\n已写入', out)


if __name__ == '__main__':
    main()
