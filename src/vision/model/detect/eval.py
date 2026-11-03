"""检测评估: 中心点误差(px) + 抖动(对同一图施加 N 次轻微扰动测预测稳定性)。

验收标准(计划书 W1 D1-2): 手掌中心抖动 ≤8px(144px 尺度)。
用法: python eval.py [--weights ../frozen/detect_fp32.pt] [--jitter-n 30]
"""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from net import DetectNet
from dataset import DetectDataset

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.qfake import calibrate_from_loader          # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def evaluate(net, dl, device, jitter_n, jitter_rng):
    net.eval()
    errs, jitters = [], []
    with torch.no_grad():
        for x, y in dl:
            p = net.predict(x.to(device)).cpu().numpy()
            y = y.numpy()
            for i in range(len(y)):
                if y[i, 0] > 0.5 and p[i, 0] > 0.5:
                    errs.append(np.hypot(p[i, 1] - y[i, 1], p[i, 2] - y[i, 2]) * 144)
                # 抖动: 同一图 jitter_n 次轻微扰动(±2px 平移), 看预测中心点标准差
                if jitter_n > 0 and y[i, 0] > 0.5:
                    batch = []
                    for _ in range(jitter_n):
                        dx, dy = int(jitter_rng.integers(-2, 3)), int(jitter_rng.integers(-2, 3))
                        xj = np.roll(x[i], (dy, dx), axis=(1, 2))
                        batch.append(xj)
                    xb = torch.from_numpy(np.stack(batch)).to(device)
                    pj = net.predict(xb).cpu().numpy()
                    pts = pj[:, 1:] * 144
                    jitters.append(float(np.mean(np.std(pts, axis=0))))
    errs = np.array(errs)
    return {
        'median_px': float(np.median(errs)) if len(errs) else None,
        'p90_px': float(np.percentile(errs, 90)) if len(errs) else None,
        'n': len(errs),
        'jitter_px': float(np.mean(jitters)) if jitters else None,
        'jitter_pass_8px': bool(jitters and np.mean(jitters) <= 8.0),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--weights', type=Path, default=ROOT / 'frozen' / 'detect_fp32.pt')
    ap.add_argument('--jitter-n', type=int, default=30)
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--int8', choices=['none', 'sym', 'affine'], default='none',
                    help='跑 INT8 假量化推理（先用本 val 集校准）；sym=对称 zp=0, affine=非对称')
    ap.add_argument('--eval-out', type=Path, default=None, help='结果 json 路径（默认 weights 同目录 detect_eval.json）')
    ap.add_argument('--int8-report', type=Path, default=None, help='每层 scale/SNR 表 json 路径')
    args = ap.parse_args()
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = DetectNet().to(device)
    net.load_state_dict(torch.load(args.weights, map_location=device))
    dl = torch.utils.data.DataLoader(
        DetectDataset('val', augment=False), batch_size=args.batch, shuffle=False,
        num_workers=args.workers, persistent_workers=args.workers > 0)
    if args.int8 != 'none':
        net, rep = calibrate_from_loader(net, dl, mode=args.int8, also_quantize=('uv_head.2',), device=device)
        if args.int8_report:
            args.int8_report.parent.mkdir(parents=True, exist_ok=True)
            args.int8_report.write_text(json.dumps(rep, indent=1), encoding='utf-8')
            print('已写入量化表', args.int8_report)
    rng = np.random.default_rng(0)
    res = evaluate(net, dl, device, args.jitter_n, rng)
    if args.int8 != 'none':
        res['int8'] = args.int8
    print(json.dumps(res, indent=1, ensure_ascii=False))
    out = args.eval_out or (args.weights.parent / 'detect_eval.json')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, indent=1), encoding='utf-8')
    print('已写入', out)


if __name__ == '__main__':
    main()
