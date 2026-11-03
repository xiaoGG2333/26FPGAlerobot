# -*- coding: utf-8 -*-
"""int8 后训练量化（PTQ）验证：float32 ONNX -> QDQ int8，量精度损失与延迟（train-pi）。

- 校准集：从训练缓存随机抽 --calib-n 张正样本 + 一半数量的负样本（域内数据，标准做法）
- 量化：QUInt8 激活 + QInt8 权重、per-channel、QuantFormat.QDQ（硬件映射最直观）
- 评估：在同一批帧上比 fp32 / int8 的 NME（全部、握拳组、p90）与 presence 判对率
- 同时导出 **热图 logits 版**（把 softmax+soft-argmax 拿出图外），
  因为 IP 里那个 soft-argmax 要么用 ReLU 质心替代、要么放 PS/PL 后处理，
  图内的 exp 是 int8 量化最脆的部位

用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/quantize_int8.py \
      --onnx results/stage1/self_run4_192/best.onnx \
      --cache results/stage1/cache_self_v3 --calib-n 256 --out results/stage1/self_run4_192/int8
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import onnx
import onnxruntime as ort
from onnxruntime.quantization import (CalibrationDataReader, QuantFormat, QuantType,
                                      quantize_static)

ROOT = Path(__file__).resolve().parents[1]


def make_reader(cache, ids, size):
    imgs = np.load(cache / 'imgs.npy', mmap_mode='r')

    class R(CalibrationDataReader):
        def __init__(self):
            self.k = 0

        def get_next(self):
            if self.k >= len(ids):
                return None
            i = ids[self.k]
            self.k += 1
            im = np.asarray(imgs[i])
            im = cv2.resize(im, (size, size), interpolation=cv2.INTER_AREA)
            x = im.astype(np.float32) / 127.5 - 1.0
            return {'img': x.transpose(2, 0, 1)[None]}

    return R()


def decode_hm(z, mode='softmax'):
    """(21,hm,hm) 热图 logits -> (21,2) ROI 归一化坐标。"""
    hm = z.shape[-1]
    Z = z.reshape(21, hm * hm).astype(np.float32)
    if mode == 'softmax':
        e = np.exp(Z - Z.max(axis=1, keepdims=True))
        w = e / e.sum(axis=1, keepdims=True)
    else:                                     # ReLU 质心：零 exp，硬件友好
        w = np.maximum(Z, 0)
        s = w.sum(axis=1, keepdims=True)
        w = w / np.where(s > 0, s, 1.0)
    g = (np.arange(hm, dtype=np.float32) + 0.5) / hm
    w = w.reshape(21, hm, hm)
    cx = (w * g[None, None, :]).sum(axis=(1, 2))
    cy = (w * g[None, :, None]).sum(axis=(1, 2))
    return np.stack([cx, cy], axis=1)


def infer_all(path, imgs, ids, size, decode='softmax'):
    so = ort.SessionOptions()
    so.intra_op_num_threads = 4
    sess = ort.InferenceSession(str(path), so, providers=['CPUExecutionProvider'])
    P, PR = [], []
    for i in ids:
        im = np.asarray(imgs[i])
        if im.shape[0] != size:
            im = cv2.resize(im, (size, size), interpolation=cv2.INTER_AREA)
        x = im.astype(np.float32) / 127.5 - 1.0
        o = sess.run(None, {'img': x.transpose(2, 0, 1)[None]})
        z = o[0]
        P.append(decode_hm(np.squeeze(z), decode) if z.ndim == 4 else z.reshape(21, 2))
        PR.append(1.0 / (1.0 + np.exp(-float(np.asarray(o[1]).ravel()[0]))))
    t0 = time.time()
    x = np.zeros((1, 3, size, size), np.float32)
    for _ in range(20):
        sess.run(None, {'img': x})
    ms = (time.time() - t0) / 20 * 1000
    return np.array(P), np.array(PR), ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--cache', default='results/stage1/cache_self_v3')
    ap.add_argument('--out', default=None)
    ap.add_argument('--calib-n', type=int, default=256)
    ap.add_argument('--eval-cache', default='results/stage1/cache_self_v1',
                    help='评估用的缓存（默认 v1，取其 test 块）')
    ap.add_argument('--seed', type=int, default=7)
    args = ap.parse_args()

    onnx_p = Path(args.onnx)
    out = Path(args.out) if args.out else onnx_p.parent / 'int8'
    out.mkdir(parents=True, exist_ok=True)
    m = onnx.load(str(onnx_p), load_external_data=False)
    d = m.graph.input[0].type.tensor_type.shape.dim
    size = int(d[2].dim_value)
    print(f'[输入] {size}x{size}   模型 {onnx_p}')

    # ---- 校准集 ----
    cache = Path(args.cache)
    pres = np.load(cache / 'presence.npy')
    pos = np.where(pres > 0.5)[0]
    neg = np.where(pres <= 0.5)[0]
    rng = np.random.default_rng(args.seed)
    n_pos = int(args.calib_n * 0.75)
    n_neg = min(len(neg), args.calib_n - n_pos)
    ids = np.concatenate([rng.choice(pos, n_pos, replace=False),
                          rng.choice(neg, n_neg, replace=False)])
    print(f'[校准集] {len(ids)} 张（正 {n_pos} / 负 {n_neg}）')

    int8_p = out / f'{onnx_p.stem}_int8.onnx'
    quantize_static(str(onnx_p), str(int8_p), make_reader(cache, ids, size),
                    quant_format=QuantFormat.QDQ, activation_type=QuantType.QUInt8,
                    weight_type=QuantType.QInt8, per_channel=True)
    print(f'[量化] {int8_p}  {int8_p.stat().st_size/1e6:.2f} MB')

    # ---- 评估：v1 test 块（386 帧，含握拳组）----
    ec = Path(args.eval_cache)
    eimgs = np.load(ec / 'imgs.npy', mmap_mode='r')
    epts = np.load(ec / 'pts.npy')
    esc = np.load(ec / 'scale.npy')
    idx = np.array(json.loads((ec / 'split.json').read_text())['test'], int)
    gt = epts[idx]
    s = esc[idx][:, None]
    TIP = [4, 8, 12, 16, 20]
    hl = np.linalg.norm(gt[:, 9] - gt[:, 0], axis=1) + 1e-6
    reach = np.linalg.norm(gt[:, TIP] - gt[:, [0]], axis=2).mean(axis=1) / hl
    fist = reach <= np.quantile(reach, 0.25)

    print(f'\n[评估] {ec} test 块 {len(idx)} 帧，握拳组 {int(fist.sum())} 帧')
    hm_model = len(ort.InferenceSession(str(onnx_p), providers=['CPUExecutionProvider'])
                    .get_outputs()[0].shape) == 4
    runs = [('fp32', onnx_p, 'softmax'), ('int8', int8_p, 'softmax')]
    if hm_model:
        runs.append(('int8+relu', int8_p, 'relu'))
        runs.append(('fp32+relu', onnx_p, 'relu'))
    rows = {}
    for tag, p, dec in runs:
        P, PR, ms = infer_all(p, eimgs, idx, size, dec)
        e = np.linalg.norm(P - gt, axis=2) / s
        n = e.mean(axis=1)
        rows[tag] = dict(nme=float(n.mean()), fist=float(n[fist].mean()),
                         p90=float(np.percentile(n, 90)), ms=float(ms),
                         pres=float(np.mean(PR)))
        print(f'  {tag:<11s} 全部 {n.mean():.4f}  握拳 {n[fist].mean():.4f}  '
              f'p90 {np.percentile(n,90):.4f}  延迟 {ms:.3f} ms')

    # ---- presence 判对率（负样本全量）----
    npres = np.load(cache / 'presence.npy')
    negids = np.where(npres <= 0.5)[0]
    cimgs = np.load(cache / 'imgs.npy', mmap_mode='r')
    posids = rng.choice(np.where(npres > 0.5)[0], 200, replace=False)
    acc = {}
    for tag, p in (('fp32', onnx_p), ('int8', int8_p)):
        _, pp, _ = infer_all(p, cimgs, posids, size)
        _, pn, _ = infer_all(p, cimgs, negids, size)
        acc[tag] = (float((pp > 0.5).mean()), float((pn < 0.5).mean()))
        rows[tag]['pos_ok'], rows[tag]['neg_ok'] = acc[tag][0], acc[tag][1]
        print(f'  {tag:<5s} presence: 正样本判对 {acc[tag][0]:.3f}  负样本判对 {acc[tag][1]:.3f}')

    best = 'int8+relu' if 'int8+relu' in rows else 'int8'
    rel = (rows[best]['nme'] / max(rows['fp32']['nme'], 1e-9) - 1) * 100
    relf = (rows[best]['fist'] / max(rows['fp32']['fist'], 1e-9) - 1) * 100
    print(f'\n[结论] 取 {best}: NME 相对变化 {rel:+.1f}%   握拳组 {relf:+.1f}%   '
          f'延迟 {rows["fp32"]["ms"]:.2f} -> {rows[best]["ms"]:.2f} ms')
    (out / 'quant_report.json').write_text(json.dumps(
        {'input': size, 'calib_n': len(ids), 'hm_model': bool(hm_model), 'rows': rows,
         'best': best, 'nme_rel_change_pct': rel, 'fist_rel_change_pct': relf}, indent=1))
    print(f'[报告] {out / "quant_report.json"}')


if __name__ == '__main__':
    main()
