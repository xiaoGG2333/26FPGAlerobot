"""窗口外/边界样本的分层分析：模型到底学到了什么分界？

动机（2026-10-15）：加入"目标手在窗口外"负样本后，窗口外拒识率只到 25.9%（目标 >=90%），
但窗口内精度在"同召回"下反而变好。要判断这是"训练没生效"还是"标签噪声决定的物理上限"，
必须按**真值离窗口边界多远**分层看拒识/误拒率：

  A. 窗口外样本：按 gt 中心离窗口距离分桶 → 拒识率是否随距离单调上升？
     （是 ⇒ 模型学到了"离得越远越该拒"，只是 HaGRID 里近边界样本的标签本身不可信）
  B. 窗口内样本：按 gt 中心离最近边界的距离分桶 → 误拒是否集中在本就贴近边界的样本？
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

OUT_BUCKETS = [(0, 8), (8, 16), (16, 32), (32, 999)]
IN_BUCKETS = [(0, 5), (5, 10), (10, 20), (20, 999)]


@torch.no_grad()
def collect(weights, device='cuda'):
    net = DetectNet().to(device).eval()
    net.load_state_dict(torch.load(weights, map_location=device))
    dl = torch.utils.data.DataLoader(DetectDataset('val', augment=False),
                                     batch_size=64, shuffle=False, num_workers=2)
    conf, u, v = [], [], []
    for x, y in dl:
        pred = net.predict(x.to(device)).cpu().numpy()
        m = y.numpy()[:, 0] > 0.5
        if not m.any():
            continue
        conf.append(pred[m, 0]); u.append(y.numpy()[m, 1]); v.append(y.numpy()[m, 2])
    conf, u, v = np.concatenate(conf), np.concatenate(u), np.concatenate(v)
    # 到窗口的距离: 在外为正(px), 在内为负(px)
    du = np.where(u > 1, (u - 1) * 144, np.where(u < 0, u * 144, 0.0))
    dv = np.where(v > 1, (v - 1) * 144, np.where(v < 0, v * 144, 0.0))
    signed = np.hypot(du, dv) * np.sign(du + dv)            # 只有一根轴会越界
    inside = np.zeros_like(signed)
    for i, (a, b) in enumerate(zip(u, v)):
        inside[i] = min(a, 1 - a, b, 1 - b) * 144           # 离最近边界的距离（内为正）
    return conf, signed, inside


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--weights', type=Path, nargs='+', required=True)
    ap.add_argument('--tags', nargs='*', default=None)
    ap.add_argument('--tag', default='boundary')
    ap.add_argument('--out-dir', type=Path, default=Path(__file__).resolve().parent)
    args = ap.parse_args()
    tags = args.tags or [w.name for w in args.weights]
    res = {}
    for w, tag in zip(args.weights, tags):
        conf, signed, inside = collect(w)
        out_m, in_m = signed > 0, signed <= 0
        rep = {'n_oob': int(out_m.sum()), 'n_in': int(in_m.sum()),
               'oob_conf_median': round(float(np.median(conf[out_m])), 4),
               'oob_by_distance': [], 'in_by_border_dist': []}
        print(f"\n[{tag}] 窗口外 {out_m.sum()} / 窗口内 {in_m.sum()}；窗口外 conf 中位 "
              f"{np.median(conf[out_m]):.4f}")
        print("  A. 窗口外样本：真值离窗口越远，拒识率是否越高？（阈值 0.5）")
        print(f"     {'距离桶(px)':>14s}{'n':>5s}{'拒识率':>9s}{'conf中位':>10s}")
        for lo, hi in OUT_BUCKETS:
            m = out_m & (signed >= lo) & (signed < hi)
            if m.sum() == 0:
                continue
            row = {'lo': lo, 'hi': hi, 'n': int(m.sum()),
                   'reject@0.5': round(float((conf[m] < 0.5).mean()), 4),
                   'conf_median': round(float(np.median(conf[m])), 4)}
            rep['oob_by_distance'].append(row)
            lab = f'{lo}-{hi}' if hi < 999 else f'{lo}+'
            print(f"     {lab:>14s}{row['n']:>5d}"
                  f"{row['reject@0.5']:>9.1%}{row['conf_median']:>10.4f}")
        print("  B. 窗口内样本：误拒是否集中在贴近边界的样本？")
        print(f"     {'离边界(px)':>14s}{'n':>5s}{'误拒率':>9s}{'conf中位':>10s}")
        for lo, hi in IN_BUCKETS:
            m = in_m & (inside >= lo) & (inside < hi)
            if m.sum() == 0:
                continue
            row = {'lo': lo, 'hi': hi, 'n': int(m.sum()),
                   'false_reject@0.5': round(float((conf[m] < 0.5).mean()), 4),
                   'conf_median': round(float(np.median(conf[m])), 4)}
            rep['in_by_border_dist'].append(row)
            lab = f'{lo}-{hi}' if hi < 999 else f'{lo}+'
            print(f"     {lab:>14s}{row['n']:>5d}"
                  f"{row['false_reject@0.5']:>9.1%}{row['conf_median']:>10.4f}")
        res[tag] = rep
    (args.out_dir / f'analyze_{args.tag}.json').write_text(
        json.dumps(res, indent=1, ensure_ascii=False), encoding='utf-8')
    print('\n已写入', args.out_dir / f'analyze_{args.tag}.json')


if __name__ == '__main__':
    main()
