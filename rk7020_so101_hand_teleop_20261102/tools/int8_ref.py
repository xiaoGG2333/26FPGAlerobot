# -*- coding: utf-8 -*-
"""int8 全定点参考实现（numpy）——写 HLS 之前把"到底算什么"钉死。

和 dump_ip_spec.py 不同：这个真的按定点把整网跑一遍，再和 onnxruntime 的 int8 输出对拍。
语义假设（由对拍结果验证）：

  1. acc_int32 = sum_c (x_int8[c] - x_zp) * w_int8[c]      权重 zp 恒 0，scale 逐输出通道
  2. bias 折进 int32：b_int = round(b_f / (sx * sw_c))
  3. 输出：y_int8 = clamp(round(acc * M) + y_zp)，M = sx * sw_c / sy
  4. 残差：out = clamp(round((a*sa + b*sb)/sy) + y_zp)
  5. 不做额外 ReLU6 反向钳位（QDQ 自身的饱和就够）

用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/int8_ref.py \
      --onnx results/stage1/self_run5_192/int8/best_hm_int8.onnx \
      --cache results/stage1/cache_cam480_v3 --n 6
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import onnx
import onnxruntime as ort
from onnx import numpy_helper

ROOT = Path(__file__).resolve().parents[1]
IDENT = ('Flatten', 'Reshape', 'Squeeze', 'Unsqueeze', 'Identity')


class G:
    def __init__(self, path):
        m = onnx.shape_inference.infer_shapes(onnx.load(str(path)))
        self.g = m.graph
        self.inits = {i.name: numpy_helper.to_array(i) for i in m.graph.initializer}
        self.prod = {o: n for n in m.graph.node for o in n.output}
        self.alias = {}
        for n in self.g.node:
            if n.op_type not in IDENT:
                continue
            src = self.xkey(n.input[0])
            dst = self.q_of(n.output[0])
            if src and dst:
                self.alias[dst[0]] = src

    def q_of(self, t):
        for n in self.g.node:
            if n.op_type == 'QuantizeLinear' and n.input[0] == t:
                sc = float(np.ravel(self.inits[n.input[1]])[0])
                zp = int(np.ravel(self.inits[n.input[2]])[0]) if len(n.input) > 2 else 0
                return n.output[0], sc, zp
        return None

    def xkey(self, t):
        n = self.prod.get(t)
        return None if n is None or n.op_type != 'DequantizeLinear' else n.input[0]

    def dq(self, t):
        n = self.prod.get(t)
        if n is None or n.op_type != 'DequantizeLinear':
            return None
        sc = np.asarray(self.inits[n.input[1]], np.float32)
        zp = self.inits[n.input[2]] if len(n.input) > 2 else np.array(0, np.uint8)
        a = self.inits.get(n.input[0])
        return sc, zp, (None if a is None else a)


def rq(acc, M, zp):
    # 激活是 uint8 非对称（zp 约 128），饱和到 [0,255]
    return np.clip(np.rint(acc * M) + zp, 0, 255).astype(np.uint8)


def resolve(key, mem, alias):
    seen = set()
    while key not in mem and key in alias and key not in seen:
        seen.add(key)
        key = alias[key]
    return mem.get(key)


def build_plan(gg):
    plan = []
    for n in gg.g.node:
        if n.op_type == 'Conv':
            wsc, _, W = gg.dq(n.input[1])
            xq, yq = gg.dq(n.input[0]), gg.q_of(n.output[0])
            at = {a.name: a for a in n.attribute}
            b = gg.dq(n.input[2])[2] if (len(n.input) > 2 and gg.dq(n.input[2])) else None
            plan.append(dict(op='conv', name=n.name, W=W.astype(np.int32) if W is not None else None,
                             wsc=np.atleast_1d(wsc).astype(np.float64),
                             bias=None if b is None else np.atleast_1d(b).astype(np.int64),
                             group=int(at['group'].i) if 'group' in at else 1,
                             stride=int(at['strides'].ints[0]) if 'strides' in at else 1,
                             pads=list(at['pads'].ints) if 'pads' in at else [0, 0, 0, 0],
                             x_scale=float(np.ravel(xq[0])[0]) if xq else None,
                             x_zp=int(np.ravel(xq[1])[0]) if xq else 0,
                             y_scale=yq[1], y_zp=yq[2],
                             x_in=gg.xkey(n.input[0]), x_out=yq[0]))
        elif n.op_type == 'Add':
            aq, bq, yq = gg.dq(n.input[0]), gg.dq(n.input[1]), gg.q_of(n.output[0])
            plan.append(dict(op='add', name=n.name,
                             a_in=gg.xkey(n.input[0]), b_in=gg.xkey(n.input[1]),
                             a_scale=float(np.ravel(aq[0])[0]), a_zp=int(np.ravel(aq[1])[0]),
                             b_scale=float(np.ravel(bq[0])[0]), b_zp=int(np.ravel(bq[1])[0]),
                             y_scale=yq[1], y_zp=yq[2], x_out=yq[0]))
        elif n.op_type == 'GlobalAveragePool':
            qq = gg.q_of(n.output[0])
            if qq is None:
                fl = next((m for m in gg.g.node
                           if m.op_type == 'Flatten' and m.input[0] == n.output[0]), None)
                qq = gg.q_of(fl.output[0]) if fl is not None else None
            xq = gg.dq(n.input[0])
            plan.append(dict(op='gap', name=n.name, x_in=gg.xkey(n.input[0]),
                             x_scale=float(np.ravel(xq[0])[0]) if xq else None,
                             x_zp=int(np.ravel(xq[1])[0]) if xq else 0,
                             y_scale=qq[1], y_zp=qq[2], x_out=qq[0]))
        elif n.op_type == 'Gemm':
            wsc, _, W = gg.dq(n.input[1])
            xq, yq = gg.dq(n.input[0]), gg.q_of(n.output[0])
            b = gg.dq(n.input[2])[2] if (len(n.input) > 2 and gg.dq(n.input[2])) else None
            plan.append(dict(op='fc', name=n.name, W=W.astype(np.int32),
                             wsc=np.atleast_1d(wsc).astype(np.float64),
                             bias=None if b is None else np.atleast_1d(b).astype(np.int64),
                             x_scale=float(np.ravel(xq[0])[0]), x_zp=int(np.ravel(xq[1])[0]),
                             y_scale=yq[1], y_zp=yq[2],
                             x_in=gg.xkey(n.input[0]), x_out=yq[0]))
    return plan


def run_plan(plan, seed, want, alias):
    mem = dict(seed)
    for L in plan:
        if L['op'] == 'conv':
            x = resolve(L['x_in'], mem, alias).astype(np.int32)
            W = L['W']
            gsz, s = L['group'], L['stride']
            pads = L['pads']
            if len(pads) == 4 and any(pads):
                # 零填充在量化域 = x_zp（不是 0）
                x = np.pad(x, ((0, 0), (pads[0], pads[2]), (pads[1], pads[3])),
                           constant_values=L['x_zp'])
            Cin, H, Wd = x.shape
            kh, kw = W.shape[2], W.shape[3]
            Ho, Wo = (H - kh) // s + 1, (Wd - kw) // s + 1
            Cout = W.shape[0]
            acc = np.zeros((Cout, Ho, Wo), np.int64)
            cg, og = Cin // gsz, Cout // gsz
            for gi in range(gsz):
                xs = x[gi * cg:(gi + 1) * cg]
                ws = W[gi * og:(gi + 1) * og]
                a = np.zeros((og, Ho, Wo), np.int64)
                for ky in range(kh):
                    for kx in range(kw):
                        a += np.tensordot(ws[:, :, ky, kx],
                                          xs[:, ky:ky + s * Ho:s, kx:kx + s * Wo:s],
                                          axes=([1], [0]))
                acc[gi * og:(gi + 1) * og] = a
            acc -= np.int64(L['x_zp']) * W.reshape(Cout, -1).sum(1)[:, None, None]
            if L['bias'] is not None:
                acc += L['bias'][:, None, None]
            M = L['x_scale'] * L['wsc'] / L['y_scale']
            M = M[:, None, None] if M.size > 1 else float(M[0])
            mem[L['x_out']] = rq(acc, M, L['y_zp'])
        elif L['op'] == 'add':
            a = resolve(L['a_in'], mem, alias).astype(np.float64)
            b = resolve(L['b_in'], mem, alias).astype(np.float64)
            y = ((a - L['a_zp']) * L['a_scale'] + (b - L['b_zp']) * L['b_scale'])                 / L['y_scale'] + L['y_zp']
            mem[L['x_out']] = np.clip(np.rint(y), 0, 255).astype(np.uint8)
        elif L['op'] == 'gap':
            a = resolve(L['x_in'], mem, alias).astype(np.float64)
            a = (a - L['x_zp']).mean(axis=(1, 2))
            y = a * (L['x_scale'] / L['y_scale']) + L['y_zp']
            mem[L['x_out']] = np.clip(np.rint(y), 0, 255).astype(np.uint8)
        elif L['op'] == 'fc':
            v = resolve(L['x_in'], mem, alias).astype(np.int64)
            acc = L['W'].astype(np.int64) @ v
            acc -= np.int64(L['x_zp']) * L['W'].sum(1)
            if L['bias'] is not None:
                acc += L['bias']
            M = L['x_scale'] * L['wsc'] / L['y_scale']
            M = M if M.size > 1 else float(M[0])
            mem[L['x_out']] = rq(acc, M, L['y_zp'])
    return {k: mem[k] for k in want if k in mem}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--cache', default=str(ROOT / 'results/stage1/cache_cam480_v3'))
    ap.add_argument('--n', type=int, default=4)
    ap.add_argument('--stride', type=int, default=60)
    args = ap.parse_args()

    gg = G(args.onnx)
    plan = build_plan(gg)
    so = ort.SessionOptions()
    so.log_severity_level = 3
    sess = ort.InferenceSession(str(args.onnx), so, providers=['CPUExecutionProvider'])
    iname = sess.get_inputs()[0].name
    S = sess.get_inputs()[0].shape[-1]

    first = next(L for L in plan if L['op'] == 'conv')
    sx0, zp0 = first['x_scale'], first['x_zp']
    last_conv = [L for L in plan if L['op'] == 'conv'][-1]
    key, yscale = last_conv['x_out'], last_conv['y_scale']
    print(f'[输入] scale={sx0:.8f} zp={zp0} -> int8 = round((pixel/127.5-1)/scale)+zp ~= pixel-128')
    print(f'[层] conv {sum(1 for L in plan if L["op"]=="conv")}  add '
          f'{sum(1 for L in plan if L["op"]=="add")}  gap/fc '
          f'{sum(1 for L in plan if L["op"] in ("gap","fc"))}')

    imgs = np.load(Path(args.cache) / 'imgs.npy', mmap_mode='r')
    tot, mx = [], 0
    for k in range(args.n):
        im = np.asarray(imgs[k * args.stride])
        if im.shape[0] != S:
            im = cv2.resize(im, (S, S), interpolation=cv2.INTER_AREA)
        xf = im.astype(np.float32) / 127.5 - 1.0
        o = sess.run(None, {iname: xf.transpose(2, 0, 1)[None]})
        xi = np.clip(np.rint(xf / sx0) + zp0, 0, 255).astype(np.uint8).transpose(2, 0, 1)
        got = run_plan(plan, {iname: xi, first['x_in']: xi}, {key}, gg.alias)
        mine = got[key].astype(np.int16)
        ref = np.rint(np.asarray(o[0])[0] / yscale) + last_conv['y_zp']  # ORT 输出是反量化后的 float
        d = np.abs(mine - ref)
        tot.append(float(d.mean()))
        mx = max(mx, int(d.max()))
        print(f'  #{k*args.stride:5d} |diff| 平均 {d.mean():.3f} 最大 {d.max():6.0f} '
              f'超 1 LSB 占比 {(d>1).mean()*100:5.1f}%  (热图 int8 范围 {mine.min()}..{mine.max()})')
    print(f'\n[结论] 平均 |diff| {np.mean(tot):.3f} LSB，最大 {mx}')


if __name__ == '__main__':
    main()
