# -*- coding: utf-8 -*-
"""在任意缓存上评估一个 ONNX（支持 coord 直出 或 热图出图）——用于 480p 实测。

用法：
  python tools/eval_cache.py --cache results/stage1/cache_cam480 \
      --onnx results/stage1/self_run4_192/int8_hm/best_hm_int8.onnx
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import live_kpt as L  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cache', required=True)
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--split', default='all')
    args = ap.parse_args()

    c = Path(args.cache)
    imgs = np.load(c / 'imgs.npy', mmap_mode='r')
    pts = np.load(c / 'pts.npy')
    pres = np.load(c / 'presence.npy')
    sc = np.load(c / 'scale.npy')
    idx = np.arange(len(pts)) if args.split == 'all' else \
        np.array(json.loads((c / 'split.json').read_text())[args.split], int)
    idx = idx[pres[idx] > 0.5]

    so = ort.SessionOptions()
    so.log_severity_level = 3
    sess = ort.InferenceSession(args.onnx, so, providers=['CPUExecutionProvider'])
    S = L.in_size_of(sess)

    P = []
    for i in idx:
        im = np.asarray(imgs[i])
        if im.shape[0] != S:
            im = cv2.resize(im, (S, S), interpolation=cv2.INTER_AREA)
        x = im.astype(np.float32) / 127.5 - 1.0
        o = sess.run(None, {'img': x.transpose(2, 0, 1)[None]})
        z = np.asarray(o[0], np.float32)
        P.append(L.decode_hm(z[0]) if z.ndim == 4 else z.reshape(21, 2))
    P = np.array(P)

    gt = pts[idx]
    s = sc[idx][:, None]
    e = np.linalg.norm(P - gt, axis=2) / s                 # (N,21) NME
    nme = e.mean(axis=1)
    TIP = [4, 8, 12, 16, 20]
    hl = np.linalg.norm(gt[:, 9] - gt[:, 0], axis=1) + 1e-6
    reach = np.linalg.norm(gt[:, TIP] - gt[:, [0]], axis=2).mean(axis=1) / hl
    q1, q4 = np.quantile(reach, 0.25), np.quantile(reach, 0.75)
    fist, open_ = reach <= q1, reach >= q4

    th = np.array([abs(L.theta_deg(a) - L.theta_deg(b)) for a, b in zip(gt, P)])
    ex = np.array([int(np.array_equal(L.ext_bits(gt[k], sc[idx][k]), L.ext_bits(P[k],
                   float(np.linalg.norm(P[k][9] - P[k][0]))))) for k in range(len(P))])

    print(f'[缓存] {args.cache}  样本 {len(idx)}  输入 {S}²')
    print(f'[掌长] ROI 归一化 p5/50/95 = {np.percentile(sc[idx],[5,50,95]).round(3)}')
    print(f'[NME]  平均 {nme.mean():.4f}  中位 {np.median(nme):.4f}  p90 {np.percentile(nme,90):.4f}')
    print(f'        <0.03 {(nme<0.03).mean()*100:.1f}%   <0.05 {(nme<0.05).mean()*100:.1f}%   '
          f'<0.10 {(nme<0.10).mean()*100:.1f}%')
    print(f'[分组] 张开(reach>={q4:.2f}) n={open_.sum()} NME {nme[open_].mean():.4f} | '
          f'握紧(reach<={q1:.2f}) n={fist.sum()} NME {nme[fist].mean():.4f}')
    print(f'[theta] 平均 {th.mean():.2f}°  中位 {np.median(th):.2f}°  p90 {np.percentile(th,90):.2f}°')
    print(f'[ext]   完全一致 {(ex==1).mean()*100:.1f}%')
    out = Path(args.onnx).parent / 'eval_cache.json'
    out.write_text(json.dumps(dict(cache=str(args.cache), n=int(len(idx)),
                                   nme_mean=float(nme.mean()), nme_median=float(np.median(nme)),
                                   nme_p90=float(np.percentile(nme, 90)),
                                   nme_open=float(nme[open_].mean()), nme_fist=float(nme[fist].mean()),
                                   theta_mean=float(th.mean()), theta_median=float(np.median(th)),
                                   ext_ok=float((ex == 1).mean())), indent=1))


if __name__ == '__main__':
    main()
