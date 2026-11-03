# -*- coding: utf-8 -*-
"""从 int8 QDQ ONNX 抽取逐层定点规格，供 HLS/RTL 当金标准参数表。

输出：
  <onnx 目录>/ip_spec.json     逐层参数（scale/zp/shape/MAC）
  --markdown 时打印 markdown 表（用于生成冻结文档）

用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/dump_ip_spec.py \
      --onnx results/stage1/self_run5_192/int8/best_hm_int8.onnx --markdown
"""
import argparse
import json
from pathlib import Path

import numpy as np
import onnx
from onnx import numpy_helper


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--out', default='')
    ap.add_argument('--markdown', action='store_true')
    args = ap.parse_args()

    m = onnx.shape_inference.infer_shapes(onnx.load(args.onnx))
    g = m.graph
    inits = {i.name: numpy_helper.to_array(i) for i in g.initializer}
    prod = {o: n for n in g.node for o in n.output}
    vi = {v.name: v for v in list(g.value_info) + list(g.output)}

    def shape_of(t):
        v = vi.get(t)
        if v is None:
            return None
        return [d.dim_value or -1 for d in v.type.tensor_type.shape.dim]

    def qparam(t):
        """t 若由 DQ 产生 -> (scale, zero_point, 量化张量名, initializer 形状 or None)。"""
        n = prod.get(t)
        if n is None or n.op_type != 'DequantizeLinear':
            return None
        name = n.input[0]
        sc = np.asarray(inits[n.input[1]], np.float32)
        zp = np.asarray(inits[n.input[2]], np.int8) if len(n.input) > 2 else np.array(0, np.int8)
        a = inits.get(name)
        return sc, zp, name, (None if a is None else a.shape)

    def q_out(t):
        for n in g.node:
            if n.op_type == 'QuantizeLinear' and n.input[0] == t:
                sc = np.asarray(inits[n.input[1]], np.float32)
                zp = np.asarray(inits[n.input[2]], np.int8) if len(n.input) > 2 else np.array(0, np.int8)
                return sc, zp
        return None

    layers, total_mac, wbytes = [], 0, 0
    seen_w = set()
    for n in g.node:
        if n.op_type == 'Conv':
            wq, xq, yq = qparam(n.input[1]), qparam(n.input[0]), q_out(n.output[0])
            at = {a.name: a for a in n.attribute}
            N, Cin, kh, kw = wq[3]
            grp = int(at['group'].i) if 'group' in at else 1
            osh = shape_of(n.output[0])
            mac = 0
            if osh and osh[2] > 0:
                mac = osh[2] * osh[3] * N * Cin * kh * kw      # Cin 已是 per-group 通道数
            rec = dict(op='Conv', name=n.name, w_int8=wq[2], w_shape=list(wq[3]),
                       kh=kh, kw=kw, group=grp,
                       stride=(list(at['strides'].ints) if 'strides' in at else [1, 1])[0],
                       pad=list(at['pads'].ints) if 'pads' in at else [0, 0, 0, 0],
                       in_shape=shape_of(n.input[0]), out_shape=osh,
                       x_scale=(float(np.ravel(xq[0])[0]) if xq else None),
                       x_zp=(int(np.ravel(xq[1])[0]) if xq else None),
                       y_scale=float(np.ravel(yq[0])[0]), y_zp=int(np.ravel(yq[1])[0]),
                       w_scale_min=float(np.min(np.atleast_1d(wq[0]))),
                       w_scale_max=float(np.max(np.atleast_1d(wq[0]))),
                       w_scale_n=int(np.size(wq[0])),
                       has_bias=len(n.input) > 2, macs=int(mac))
            layers.append(rec)
            total_mac += mac
            if wq[2] not in seen_w:
                seen_w.add(wq[2])
                wbytes += int(inits[wq[2]].size)
        elif n.op_type == 'Add':
            xq, yq, bq = qparam(n.input[0]), q_out(n.output[0]), qparam(n.input[1])
            if not (xq and yq):
                continue
            layers.append(dict(op='ResAdd', name=n.name, out_shape=shape_of(n.output[0]),
                               x_scale=float(np.ravel(xq[0])[0]), x_zp=int(np.ravel(xq[1])[0]),
                               b_scale=float(np.ravel(bq[0])[0]) if bq else None,
                               b_zp=int(np.ravel(bq[1])[0]) if bq else None,
                               y_scale=float(np.ravel(yq[0])[0]), y_zp=int(np.ravel(yq[1])[0])))
        elif n.op_type == 'GlobalAveragePool':
            xq, yq = qparam(n.input[0]), q_out(n.output[0])
            layers.append(dict(op='GAP', name=n.name, in_shape=shape_of(n.input[0]),
                               x_scale=float(np.ravel(xq[0])[0]) if xq else None,
                               y_scale=float(np.ravel(yq[0])[0]) if yq else None))
        elif n.op_type == 'Gemm':
            wq, xq, yq = qparam(n.input[1]), qparam(n.input[0]), q_out(n.output[0])
            if wq is None:
                continue
            M = int(wq[3][0])
            mac = M * int(wq[3][1])
            layers.append(dict(op='Gemm', name=n.name, w_shape=list(wq[3]), w_int8=wq[2],
                               x_scale=float(np.ravel(xq[0])[0]) if xq else None,
                               y_scale=float(np.ravel(yq[0])[0]) if yq else None,
                               w_scale_min=float(np.min(np.atleast_1d(wq[0]))),
                               w_scale_max=float(np.max(np.atleast_1d(wq[0]))),
                               w_scale_n=int(np.size(wq[0])), macs=mac))
            total_mac += mac
            if wq[2] not in seen_w:
                seen_w.add(wq[2])
                wbytes += int(inits[wq[2]].size)

    peak = 0
    for r in layers:
        s = r.get('out_shape')
        if s and s[0] > 0:
            peak = max(peak, int(np.prod(s)))
    summary = dict(weight_bytes=wbytes, total_macs=int(total_mac),
                   peak_activation_bytes=peak, n_conv=sum(1 for r in layers if r['op'] == 'Conv'))
    print(f'[汇总] Conv {summary["n_conv"]}  总 MAC {total_mac/1e6:.1f}M  '
          f'权重 {wbytes/1024:.1f} KB  单层最大激活 {peak/1024:.1f} KB')

    if args.markdown:
        print('\n| # | 层 | 类型 | 权重 | s | g | 输出形状 | 输出 scale | MACs |')
        print('|---|---|---|---|---|---|---|---|---|')
        k = 0
        for r in layers:
            if r['op'] == 'Conv':
                k += 1
                kind = 'dw3x3' if r['group'] > 1 else ('pw1x1' if r['kh'] == 1 else 'conv')
                print(f'| {k} | `{r["name"]}` | {kind} | {r["w_shape"]} | {r["stride"]} | '
                      f'{r["group"]} | {r["out_shape"]} | {r["y_scale"]:.5f} | {r["macs"]:,} |')
            elif r['op'] == 'ResAdd':
                print(f'| - | `{r["name"]}` | 残差相加 | - | - | - | {r["out_shape"]} | '
                      f'{r["y_scale"]:.5f} | - |')
            elif r['op'] == 'Gemm':
                print(f'| - | `{r["name"]}` | FC | {r["w_shape"]} | - | - | - | '
                      f'{r["y_scale"]:.5f} | {r["macs"]:,} |')

    out = Path(args.out) if args.out else Path(args.onnx).parent / 'ip_spec.json'
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(dict(onnx=str(args.onnx), summary=summary, layers=layers),
                              indent=1, ensure_ascii=False))
    print(f'[写出] {out}')


if __name__ == '__main__':
    main()
