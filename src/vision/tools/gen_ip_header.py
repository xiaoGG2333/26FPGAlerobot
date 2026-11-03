# -*- coding: utf-8 -*-
"""把 int8 QDQ ONNX 转成 HLS 可直接用的 C 头文件 + 权重二进制。

产出（默认写到 fpga/ip/kpt_net/data/）：
  kpt_layers.h   KptLayer 表（每层：类型/通道/尺寸/stride/pad/group/权重偏移/bias 偏移/mult 偏移/scale/zp）
  kpt_params.h   KPT_WEIGHT_BYTES / KPT_BIAS_WORDS / KPT_LAYER_COUNT 等常量
  kpt_weights.bin  int8 权重 + int32 bias + int32 mult + int8 shift 的拼接体

定点规则（已用 tools/int8_ref.py 与 onnxruntime 对拍验证，平均差 0.32 LSB）：
  激活 uint8 非对称（zp 逐层；ReLU6 输出层 zp=0、scale=max/255，饱和即钳位）
  权重 int8，zp=0，逐输出通道 scale
  bias 为 int32（图里已按 sx*sw 量化）
  y = clamp( (acc*mult + (1<<(30+shift))) >> (31+shift) + y_zp , 0, 255 )
  M = sx*sw_c/sy，用 (mult, shift) 定点表示

用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/gen_ip_header.py \
      --onnx results/stage1/self_run5_192/int8/best_hm_int8.onnx
"""
import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import int8_ref as R  # noqa: E402

KIND_CONV = 0   # 普通卷积（首层 3x3 s2）
KIND_PW = 1     # 1x1 逐点
KIND_DW = 2     # 3x3 逐通道
KIND_ADD = 3
KIND_GAP = 4
KIND_FC = 5


def quant_mult_shift(m):
    """把 (0,1) 的实数 M 拆成 (mult:int32, shift:int)，使 M ≈ mult / 2^(31+shift)。"""
    if not (m > 0):
        return 0, 0
    shift = 0
    while m < 0.5:
        m *= 2.0
        shift += 1
    while m >= 1.0:
        m /= 2.0
        shift -= 1
    return int(round(m * (1 << 31))), int(shift)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx', required=True)
    ap.add_argument('--outdir', default=str(ROOT / 'fpga/ip/kpt_net/data'))
    args = ap.parse_args()

    gg = R.G(args.onnx)
    plan = R.build_plan(gg)
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)

    wbuf, bbuf, mbuf, sbuf = [], [], [], []
    w_off = b_off = m_off = s_off = 0
    layers = []
    ch, hh, ww = 3, 192, 192           # 逐层跟踪激活形状
    max_act = ch * hh * ww             # 最大激活张量（决定 DDR scratch）
    max_skip = 0                       # 最大跳跃支路
    for L in plan:
        if L['op'] == 'conv':
            W = L['W'].astype(np.int8)
            cout, cg, kh, kw = W.shape
            grp = int(L['group'])
            kind = KIND_CONV if grp == 1 and kh == 3 else (KIND_DW if grp > 1 else KIND_PW)
            layers.append(dict(kind=kind, name=L['name'],
                               cin=int(cg * grp), cg=int(cg),
                               cout=int(cout), group=grp, stride=int(L['stride']),
                               pad_l=int(L['pads'][1]), pad_t=int(L['pads'][0]),
                               pad_r=int(L['pads'][3]), pad_b=int(L['pads'][2]),
                               x_zp=int(L['x_zp']), y_zp=int(L['y_zp']),
                               x_scale=float(L['x_scale']), y_scale=float(L['y_scale']),
                               w_off=w_off, w_n=int(W.size),
                               b_off=b_off, b_n=int(cout),
                               m_off=m_off, s_off=s_off, ch_n=int(cout),
                               wsc=L['wsc']))
            k = 1 if kind == KIND_PW else 3
            hout = (hh + L['pads'][0] + L['pads'][2] - k) // L['stride'] + 1
            wout = (ww + L['pads'][1] + L['pads'][3] - k) // L['stride'] + 1
            # block 入口（扩张逐点层）要把当前张量存进 skip 缓冲
            if kind == KIND_PW and cout > ch:
                max_skip = max(max_skip, ch * hh * ww)
            if cout == 21:          # 热图支路：吃当前张量，不改主干形状
                max_act = max(max_act, 21 * hout * wout)
            else:
                hh, ww, ch = hout, wout, int(cout)
                max_act = max(max_act, ch * hh * ww)
            wbuf.append(W.reshape(-1))
            w_off += W.size
            b = (L['bias'] if L['bias'] is not None
                 else np.zeros(cout, np.int64)).astype(np.int32)
            bbuf.append(b)
            b_off += cout
            M = L['x_scale'] * L['wsc'] / L['y_scale']
            ms = [quant_mult_shift(float(x)) for x in np.atleast_1d(M)]
            mbuf.append(np.array([x[0] for x in ms], np.int32))
            sbuf.append(np.array([x[1] for x in ms], np.int8))
            m_off += cout
            s_off += cout
        elif L['op'] == 'add':
            QS = 20
            am = int(round(L['a_scale'] / L['y_scale'] * (1 << QS)))
            bm = int(round(L['b_scale'] / L['y_scale'] * (1 << QS)))
            layers.append(dict(kind=KIND_ADD, name=L['name'], cin=int(ch), x_zp=0,
                               am=am, bm=bm, qsh=QS,
                               y_zp=int(L['y_zp']),
                               a_scale=float(L['a_scale']), a_zp=int(L['a_zp']),
                               b_scale=float(L['b_scale']), b_zp=int(L['b_zp']),
                               y_scale=float(L['y_scale'])))
        elif L['op'] == 'gap':
            QS = 20
            hw = hh * ww
            am = int(round(L['x_scale'] / L['y_scale'] / hw * (1 << QS)))   # 已把 /hw 折进去
            hh = ww = 1
            max_act = max(max_act, ch)
            layers.append(dict(kind=KIND_GAP, name=L['name'], cin=int(ch),
                               am=am, bm=0, qsh=QS, hw=hw,
                               x_zp=int(L['x_zp']), y_zp=int(L['y_zp']),
                               x_scale=float(L['x_scale']), y_scale=float(L['y_scale'])))
        elif L['op'] == 'fc':
            W = L['W'].astype(np.int8)
            cout, cin = W.shape
            layers.append(dict(kind=KIND_FC, name=L['name'], cin=int(cin), cg=int(cin),
                               cout=int(cout), group=1, stride=1,
                               pad_l=0, pad_t=0, pad_r=0, pad_b=0,
                               x_zp=int(L['x_zp']), y_zp=int(L['y_zp']),
                               x_scale=float(L['x_scale']), y_scale=float(L['y_scale']),
                               w_off=w_off, w_n=int(W.size), b_off=b_off, b_n=int(cout),
                               m_off=m_off, s_off=s_off, ch_n=int(cout), wsc=L['wsc']))
            max_act = max(max_act, int(cout))
            ch = int(cout)
            wbuf.append(W.reshape(-1))
            w_off += W.size
            bbuf.append((L['bias'] if L['bias'] is not None
                         else np.zeros(cout, np.int64)).astype(np.int32))
            b_off += cout
            M = L['x_scale'] * L['wsc'] / L['y_scale']
            ms = [quant_mult_shift(float(x)) for x in np.atleast_1d(M)]
            mbuf.append(np.array([x[0] for x in ms], np.int32))
            sbuf.append(np.array([x[1] for x in ms], np.int8))
            m_off += cout
            s_off += cout

    Wb = np.concatenate(wbuf).astype(np.int8).tobytes()
    Bb = np.concatenate(bbuf).astype(np.int32).tobytes()
    Mb = np.concatenate(mbuf).astype(np.int32).tobytes()
    Sb = np.concatenate(sbuf).astype(np.int8).tobytes()
    blob = Wb + Bb + Mb + Sb
    (out / 'kpt_weights.bin').write_bytes(blob)

    # 段偏移（都从文件头算）
    o_b = len(Wb)
    o_m = o_b + len(Bb)
    o_s = o_m + len(Mb)
    lines = ['// 自动生成，勿手改（tools/gen_ip_header.py）',
             f'// 源模型：{args.onnx}',
             '#pragma once', '#include <stdint.h>', '',
             'static const int KPT_WEIGHT_BYTES = %d;' % len(Wb),
             'static const int KPT_BIAS_WORDS   = %d;' % (len(Bb) // 4),
             'static const int KPT_BLOB_BYTES   = %d;' % len(blob),
             'static const int KPT_OFF_BIAS     = %d;' % o_b,
             'static const int KPT_OFF_MULT     = %d;' % o_m,
             'static const int KPT_OFF_SHIFT    = %d;' % o_s,
             'static const int KPT_LAYER_COUNT  = %d;' % len(layers), '',
             '// 4 个独立权重口：HLS 不允许在 m_axi 指针上做 reinterpret_cast，',
             '// 所以 int8 权重 / int32 bias / int32 mult / int8 shift 各走一个口',
             'static const int KPT_W_BYTES    = %d;' % o_b,
             'static const int KPT_B_WORDS    = %d;' % (len(Bb) // 4),
             'static const int KPT_M_WORDS    = %d;' % (len(Mb) // 4),
             'static const int KPT_S_BYTES    = %d;' % len(Sb),
             'static const int KPT_BLOB_WORDS = %d;' % (len(blob) // 4), '',
             '// 激活缓冲（外部 DDR，V2.5 起不再用 std::vector）',
             'static const int KPT_MAX_ACT   = %d;' % max_act,
             'static const int KPT_MAX_SKIP  = %d;' % max_skip,
             'static const int KPT_SCRATCH_A    = 0;',
             'static const int KPT_SCRATCH_B    = KPT_MAX_ACT;',
             'static const int KPT_SCRATCH_SKIP = KPT_MAX_ACT * 2;',
             'static const int KPT_SCRATCH_BYTES = KPT_MAX_ACT * 2 + KPT_MAX_SKIP;', '']
    (out / 'kpt_params.h').write_text('\n'.join(lines) + '\n', encoding='utf-8')

    L2 = ['// 自动生成，勿手改（tools/gen_ip_header.py）', '#pragma once',
          '#include "kpt_params.h"', '',
          'struct KptLayer {',
          '  uint8_t  kind;      // 0=conv3x3 1=pointwise 2=depthwise3x3 3=resadd 4=gap 5=fc',
          '  uint8_t  stride;',
          '  uint8_t  group_log; // dw 层 = log2(groups)，其余 0',
          '  uint16_t cin, cout;',
          '  int16_t  x_zp, y_zp, a_zp, b_zp;',
          '  int16_t  pad_t, pad_l, pad_b, pad_r;',
          '  uint32_t w_off, w_n, b_off, m_off, s_off;',
          '  uint16_t ch_n;',
          '  int32_t  am, bm;    // 残差/GAP 的 Q20 系数（非这两类层为 0）',
          '  int8_t   qsh;       // 上面系数的定点小数位数（=20）',
          '  float    x_scale, y_scale, b_scale;',
          '};', '', 'static const KptLayer KPT_LAYERS[KPT_LAYER_COUNT] = {']

    def row(kind, stride, gl, cin, cout, x_zp, y_zp, a_zp, b_zp, pt, pl, pb, pr,
            w_off, w_n, b_off, m_off, s_off, ch_n, am, bm, qsh, xs, ys, bs, name):
        return ('  {%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,'
                '%d,%d,%d,%.9g,%.9g,%.9g},  // %s'
                % (kind, stride, gl, cin, cout, x_zp, y_zp, a_zp, b_zp, pt, pl, pb, pr,
                   w_off, w_n, b_off, m_off, s_off, ch_n, am, bm, qsh, xs, ys, bs, name))

    for L in layers:
        k = L['kind']
        if k in (KIND_CONV, KIND_PW, KIND_DW, KIND_FC):
            gl = int(np.log2(max(1, L['group']))) if k == KIND_DW else 0
            L2.append(row(k, L['stride'], gl, L['cin'], L['cout'], L['x_zp'], L['y_zp'],
                          0, 0, L.get('pad_t', 0), L.get('pad_l', 0), L.get('pad_b', 0),
                          L.get('pad_r', 0), L['w_off'], L['w_n'], L['b_off'], L['m_off'],
                          L['s_off'], L['ch_n'], 0, 0, 0,
                          L['x_scale'], L['y_scale'], 0.0, L['name']))
        elif k == KIND_ADD:
            L2.append(row(k, 1, 0, L['cin'], 0, 0, L['y_zp'], L['a_zp'], L['b_zp'],
                          0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                          L['am'], L['bm'], L['qsh'],
                          L['a_scale'], L['y_scale'], L['b_scale'], L['name']))
        else:  # GAP
            L2.append(row(k, 1, 0, L['cin'], 0, L['x_zp'], L['y_zp'], 0, 0, 0, 0, 0, 0,
                          0, 0, 0, 0, 0, 0,
                          L['am'], 0, L['qsh'],
                          L['x_scale'], L['y_scale'], 0.0, L['name']))
    L2.append('};')
    (out / 'kpt_layers.h').write_text('\n'.join(L2) + '\n', encoding='utf-8')

    # ------------------------------------------------------------------
    # 方案 B：逐层展开的直写调度（data/kpt_net_gen.h）
    #   目的：层表不进综合，每个循环边界都是字面量，HLS 不用再在一张巨型 CFG 上
    #   带动态边界调度（那一步会吃掉几十分钟）。注意 c/h/w 和 cur/nxt/skip 的
    #   乒乓在这里就全部算成常量。
    # ------------------------------------------------------------------
    g = []
    gch, ghh, gww = 3, 192, 192
    cur, nxt = 0, max_act
    skipb = max_act * 2

    def emit(fn, toks, i, nm):
        # toks 里可以混字符串（w / sc）和整数常量，避免手数 %d 个数
        body = ', '.join(str(t) for t in toks)
        g.append('  %s(%s);  // L%02d %s' % (fn, body, i, str(nm).strip('/')[:40]))

    for i, L in enumerate(layers):
        k = L['kind']
        nm = L.get('name', '')
        if k in (KIND_CONV, KIND_PW, KIND_DW):
            kk = 1 if k == KIND_PW else 3
            s = L['stride']
            hout = (ghh + L['pad_t'] + L['pad_b'] - kk) // s + 1
            wout = (gww + L['pad_l'] + L['pad_r'] - kk) // s + 1
            if k == KIND_PW and L['cout'] > gch:
                emit('k_copy', ['sc', cur, skipb, gch * ghh * gww], i, '存跳跃支路')
            if k == KIND_DW:
                emit('k_dw', ['w', 'sc', cur, nxt, L['cout'], s, L['x_zp'], L['y_zp'],
                              L['w_off'], L['b_off'], L['m_off'], L['s_off'],
                              ghh, gww, hout, wout, L['pad_t'], L['pad_l']], i, nm)
            elif k == KIND_PW:
                emit('k_pw', ['w', 'sc', cur, nxt, L['cin'], L['cout'], L['x_zp'], L['y_zp'],
                              L['w_off'], L['b_off'], L['m_off'], L['s_off'], ghh * gww], i, nm)
            else:
                emit('k_c3', ['w', 'sc', cur, nxt, L['cin'], L['cout'], s, L['x_zp'], L['y_zp'],
                              L['w_off'], L['b_off'], L['m_off'], L['s_off'],
                              ghh, gww, hout, wout, L['pad_t'], L['pad_l']], i, nm)
            if L['cout'] == 21:                     # 热图支路，不吃掉主干
                emit('k_hm', ['sc', nxt, L['cout'] * hout * wout], i, '热图搬到片上 HM_BUF')
            else:
                cur, nxt = nxt, cur
                gch, ghh, gww = L['cout'], hout, wout
        elif k == KIND_ADD:
            emit('k_add', ['sc', skipb, cur, nxt, gch * ghh * gww,
                           L['a_zp'], L['b_zp'], L['y_zp'],
                           L['am'], L['bm'], L['qsh']], i, nm)
            cur, nxt = nxt, cur
        elif k == KIND_GAP:
            emit('k_gap', ['sc', cur, gch, ghh * gww, L['x_zp'], L['y_zp'],
                           L['am'], L['qsh']], i, nm)
            ghh = gww = 1
        else:
            which = 0 if L['cout'] == 256 else (1 if L['cout'] == 1 else 2)
            emit('k_fc', ['w', L['w_off'], L['cin'], L['cout'], L['x_zp'], L['y_zp'],
                          L['b_off'], L['m_off'], L['s_off'], which], i, nm)

    G = ['// 自动生成，勿手改（tools/gen_ip_header.py）—— 方案 B：逐层展开的直写调度',
         '// 层表不进综合：每层的循环边界、激活偏移、乒乓关系都在生成期算成字面量。',
         '#pragma once', '#include <stdint.h>', '',
         'static void kpt_body(uint8_t* sc, const uint8_t* w) {',
         '#pragma HLS INLINE'] + g + ['}']
    (out / 'kpt_net_gen.h').write_text('\n'.join(G) + '\n', encoding='utf-8')

    import collections
    kk = collections.Counter(L['kind'] for L in layers)
    print(f'[层] 总 {len(layers)}  conv {kk[0]}  pw {kk[1]}  dw {kk[2]}  add {kk[3]}  '
          f'gap {kk[4]}  fc {kk[5]}')
    print(f'[权重] {len(Wb)/1024:.1f} KB  bias {len(Bb)/1024:.1f} KB  mult {len(Mb)/1024:.1f} KB  '
          f'shift {len(Sb)} B  合计 {len(blob)/1024:.1f} KB')
    print(f'[写出] {out/"kpt_layers.h"}  {out/"kpt_params.h"}  {out/"kpt_weights.bin"}  {out/"kpt_net_gen.h"}')
    print('\n前 6 层：')
    for L, h in list(zip(layers, []))[:6] or []:
        pass
    for L in layers[:6]:
        print('  ', {k: L[k] for k in L if k in ('kind', 'cin', 'cout', 'stride', 'group', 'w_n', 'ch_n')})


if __name__ == '__main__':
    main()
