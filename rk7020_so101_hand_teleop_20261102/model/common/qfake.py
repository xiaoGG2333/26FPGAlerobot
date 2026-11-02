"""INT8 PTQ 假量化模拟器（红线 3「INT8 未实测」的实测工具）。

实现口径与 `docs/网络结构清单.md` §5 完全一致：
  - **BN 折叠**进前一层 conv（推理时无独立 BN 算子），bias = (beta - mean*scale)
  - **权重**：逐输出通道对称量化，`s_w = max|w_c|/127`，zero_point=0
  - **激活**：逐张量量化；默认 `sym`（对称，zp=0，`s = pct_{99.9}(|x|)/127`），
    可选 `affine`（非对称，覆盖 min~max，uint8 风格，适合 ReLU 后的非负激活）
  - **累加**：int8×int8 → int32 **精确累加**（用 float32 计算整数值的乘加模拟：单点累加项数 ≤ ~200，
    和 < 2^24，float32 表示精确，与真 int32 累加等价）
  - **bias**：浮点加（int32 累加结果 ×(s_x·s_w) 后加浮点 bias）
  - **量化 ReLU**：ReLU 在 int8 域 clip（对称即 clip 到 [0,127]）
  - **softmax / soft-argmax / sigmoid**：浮点计算（与清单一致）；热图可额外量化一拍（清单：热图 int8 交后端）

用法：
    net = DetectNet().eval(); net.load_state_dict(...)
    rep = prepare_int8(net, calib_batches, mode='sym', also_quantize=('uv_head.2',))
    # 之后 net 即为「INT8 推理」；rep 可直接 json 落盘（每层 scale / SNR）
"""
import copy

import torch
import torch.nn as nn

PCT = 0.999                      # 校准分位（去离群）
ACT_BITS, W_BITS = 8, 8
_QMAX = 2 ** (W_BITS - 1) - 1    # 127


# ---------------------------------------------------------------- BN 折叠
def fold_bn(net):
    """返回 BN 已折进前一层 conv 的副本（BN 位置替换为 Identity，模块名保持不变）。"""
    net = copy.deepcopy(net).eval()
    todo = []
    for name, mod in list(net.named_modules()):
        if not isinstance(mod, nn.BatchNorm2d):
            continue
        pname, _, attr = name.rpartition('.')
        parent = net.get_submodule(pname) if pname else net
        if isinstance(parent, nn.Sequential):
            i = int(attr)
            prev = parent[i - 1]
            if isinstance(prev, nn.Conv2d):
                todo.append((parent, i, prev, mod))
    for parent, i, conv, bn in todo:
        w = conv.weight.data
        b = (conv.bias.data.clone() if conv.bias is not None
             else torch.zeros(w.size(0), dtype=w.dtype, device=w.device))
        s = bn.weight.data / torch.sqrt(bn.running_var + bn.eps)
        conv.weight.data = w * s.view(-1, 1, 1, 1)
        conv.bias = nn.Parameter((b - bn.running_mean) * s + bn.bias.data)
        parent[i] = nn.Identity()
    return net


def conv_layers(net):
    return [(n, m) for n, m in net.named_modules() if isinstance(m, nn.Conv2d)]


# ---------------------------------------------------------------- 权重量化
def quantize_weights_(net):
    """逐输出通道对称量化（就地），返回 {layer: s_w tensor}。"""
    scales = {}
    for name, m in conv_layers(net):
        w = m.weight.data
        c = w.size(0)
        amax = w.reshape(c, -1).abs().amax(dim=1).clamp_min(1e-8)
        s = amax / _QMAX
        q = torch.round(w / s.view(-1, 1, 1, 1)).clamp(-_QMAX - 1, _QMAX)
        m.weight.data = q * s.view(-1, 1, 1, 1)
        scales[name] = s.detach().cpu()
    return scales


def _qparams(x, mode):
    """由样本张量求 (scale, zero_point, lo, hi)。

    sym:  zp=0，s = pct(|x|)/127
    affine: 把 [lo,hi] 映射到 int8 的 [-128,127]（**共 255 码**）→ zp = round(-lo/s) - 128。
            注意 zp 必须按 int8 的半开区间算：写成 round(-lo/s)（uint8 的 0..255 约定）
            会让正半轴只剩 127 码，ReLU 型激活会被截断一半（2026-10-14 踩过）。
    """
    x = x.float()
    if mode == 'sym':
        amax = float(torch.quantile(x.abs(), PCT))
        return max(amax / _QMAX, 1e-8), 0.0, -max(amax, 1e-8), max(amax, 1e-8)
    lo = float(torch.quantile(x, (1 - PCT) / 2))
    hi = float(torch.quantile(x, 1 - (1 - PCT) / 2))
    if hi <= lo:
        hi = lo + 1e-6
    s = max((hi - lo) / 255.0, 1e-8)
    zp = float(min(127.0, max(-128.0, round(-lo / s) - 128)))
    return s, zp, lo, hi


def _quantize(x, s, zp):
    return (torch.round(x / s) + zp).clamp(-128, 127)


def _dequant(x, s, zp):
    """注意：入参是**浮点**张量，本函数自己会先量化再反量化（往返）。
    不要传已经量化过的值进来 —— 会变成二次量化（2026-10-15 踩坑：QAT 的 STE 写错这一步，
    网络输出直接从 5.5px 崩到 38px）。"""
    return (_quantize(x, s, zp) - zp) * s


def _stats_of(x, mode):
    x = x.float()
    s, zp, lo, hi = _qparams(x, mode)
    xq = _quantize(x, s, zp)
    err = (xq - zp) * s - x
    snr = 10 * torch.log10((x ** 2).mean() / err.pow(2).mean().clamp_min(1e-20)).item()
    return {
        's': round(s, 8), 'zp': zp, 'lo': round(lo, 6), 'hi': round(hi, 6),
        'min': round(float(x.min()), 6), 'max': round(float(x.max()), 6),
        'absmax': round(float(x.abs().max()), 6),
        # clip 比例 = 落到可表示区间 [lo,hi] 之外的比例（不是"贴边界码"的比例）
        'clip_frac': round(float(((x < lo) | (x > hi)).float().mean()), 6),
        'snr_db': round(snr, 2),
    }


# ---------------------------------------------------------------- 校准
class _Collector:
    """收集每个 conv 的输入激活样本；对 outputs 里指定的层额外收集其输出。"""

    def __init__(self, net, cap=100_000, outputs=(), seed=1234):
        self.cap = cap
        self.seed = seed
        self.outputs = set(outputs)
        self.acc_in, self.acc_out = {}, {}
        self.hooks = []
        for name, m in conv_layers(net):
            self.hooks.append(m.register_forward_pre_hook(self._mk(name, self.acc_in)))
            if name in self.outputs:
                self.hooks.append(m.register_forward_hook(self._mk(name, self.acc_out)))

    def _mk(self, name, store):
        def hook(mod, inp, out=None):
            x = (inp[0] if out is None else out).detach().reshape(-1)
            n = x.numel()
            if n > self.cap:
                # 固定种子抽稀 → 量化表可复现（否则每层分位估计会小幅抖动，精度 ±0.2pt）
                g = torch.Generator(device=x.device).manual_seed(self.seed)
                x = x[torch.randperm(n, device=x.device, generator=g)[: self.cap]]
            store.setdefault(name, []).append(x.float().cpu())
        return hook

    def close(self):
        for h in self.hooks:
            h.remove()
        self.hooks = []

    def stats(self, store, mode):
        return {name: _stats_of(torch.cat(chunks), mode) for name, chunks in store.items()}


# ---------------------------------------------------------------- 启用假量化
def _ste(x, s, zp):
    """直通估计：前向 == 量化-反量化，反向把梯度原样传回 x。仅 QAT 用。

    普通 `_dequant(_quantize(x))` 里 round/clamp 的导数几乎处处为 0 → QAT 学不动，
    必须这样写。前向数值与不加 STE 时完全一致，所以评测口径不变。
    """
    dq = _dequant(x, s, zp)          # 一次往返即可；再套一层会二次量化
    return x + (dq - x).detach()


def _pre_hook(s, zp, ste=False):
    def hook(mod, inp):
        y = _ste(inp[0], s, zp) if ste else _dequant(inp[0], s, zp)
        return (y, *inp[1:])
    return hook


def _post_hook(s, zp, ste=False):
    def hook(mod, inp, out):
        return _ste(out, s, zp) if ste else _dequant(out, s, zp)
    return hook


def enable_fake_quant_(net, act_scales, out_scales=None, ste=False):
    """给每个 conv 挂输入量化前钩；out_scales 里的层额外量化输出（如热图）。

    ste=True 时用直通估计（QAT 微调必须用；纯评测用 False，两者前向数值一致）。
    """
    for name, m in conv_layers(net):
        if name in act_scales:
            m.register_forward_pre_hook(
                _pre_hook(act_scales[name]['s'], act_scales[name]['zp'], ste))
    for name, p in (out_scales or {}).items():
        m = dict(net.named_modules()).get(name)
        if m is not None:
            m.register_forward_hook(_post_hook(p['s'], p['zp'], ste))
    return net


# ---------------------------------------------------------------- 主入口
def prepare_int8(net, calib_batches, mode='sym', also_quantize=(), verbose=True, ste=False):
    """由浮点 net 造出 INT8 假量化模型。calib_batches: 张量 或 (x, y) 的可迭代对象。

    返回 **(int8_net, report)**：注意 BN 折叠会产生新副本，必须用返回的 net 做推理。

    ste=True: 假量化用直通估计（QAT 微调用）；False: 普通评测口径。两者前向数值一致。
    """
    net = fold_bn(net)
    s_w = quantize_weights_(net)
    col = _Collector(net, outputs=also_quantize)
    n_used, n_batches = 0, 0
    with torch.no_grad():
        for b in calib_batches:
            x = b[0] if isinstance(b, (tuple, list)) else b
            net(x)
            n_used += int(x.size(0))
            n_batches += 1
    col.close()
    layers = col.stats(col.acc_in, mode)
    layers_out = col.stats(col.acc_out, mode)
    for name in layers:
        sw = s_w[name]
        layers[name].update({'s_w_min': round(float(sw.min()), 8),
                             's_w_mean': round(float(sw.mean()), 8),
                             's_w_max': round(float(sw.max()), 8), 'n_ch': int(sw.numel())})
    enable_fake_quant_(net, layers, out_scales=layers_out, ste=ste)
    rep = {'mode': mode, 'pct': PCT, 'act_bits': ACT_BITS, 'w_bits': W_BITS,
           'calib_samples': n_used, 'calib_batches': n_batches,
           'n_layers': len(layers), 'also_quantize': list(also_quantize),
           'layers': layers, 'layers_out': layers_out}
    if verbose:
        snrs = sorted(v['snr_db'] for v in layers.values())
        clips = max(v['clip_frac'] for v in layers.values())
        print(f'[int8] mode={mode} 校准 {n_used} 张/{n_batches} 批，{len(layers)} 层 | '
              f'输入量化 SNR 最低 {snrs[0]:.1f} / 中位 {snrs[len(snrs)//2]:.1f} dB | 最大 clip 比例 {clips:.4%}')
    return net, rep


# ---------------------------------------------------------------- 逐层误差预算表
@torch.no_grad()
def layer_snr(net_fp, net_q, x):
    """同一批输入下逐层输出 vs 浮点输出的 SNR(dB) 与余弦相似度（HLS 逐层误差预算表）。"""
    outs = {}

    def rec(net, tag):
        hs = [m.register_forward_hook(
            lambda mod, inp, out, n=f'{tag}:{name}': outs.__setitem__(n, out.detach().float().cpu()))
            for name, m in conv_layers(net)]
        net(x)
        for h in hs:
            h.remove()

    rec(net_fp, 'fp')
    rec(net_q, 'q')
    rows = {}
    for k in list(outs):
        if not k.startswith('fp:'):
            continue
        name = k[3:]
        a, b = outs[k].reshape(-1), outs['q:' + name].reshape(-1)
        p, e = (a ** 2).mean(), (b - a).pow(2).mean()
        rows[name] = {'out_snr_db': round(10 * torch.log10(p / e.clamp_min(1e-20)).item(), 2),
                      'cos': round(float(torch.nn.functional.cosine_similarity(a, b, dim=0)), 5)}
    return rows


def load_net_state(net, weights):
    """加载权重（兼容 fp32 state_dict）。"""
    sd = torch.load(weights, map_location='cpu')
    net.load_state_dict(sd)
    return net


def calibrate_from_loader(net, loader, mode='sym', also_quantize=(), device='cuda', verbose=True):
    """用 DataLoader(x,y) 校准 → 返回 (int8_net, report)。"""
    return prepare_int8(net, ((x.to(device),) for x, y in loader),
                        mode=mode, also_quantize=also_quantize, verbose=verbose)
