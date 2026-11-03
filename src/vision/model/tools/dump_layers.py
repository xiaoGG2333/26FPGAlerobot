"""导出《网络结构清单》用的逐层表（算子/shape/MACs/参数），供 docs/网络结构清单.md 引用。

用法（在工程根）:
    D:/Anaconda3/envs/fpga_amd-pi/python.exe model/tools/dump_layers.py
    D:/Anaconda3/envs/fpga_amd-pi/python.exe model/tools/dump_layers.py --markdown > docs/_layers.md
"""
import argparse
import sys
from pathlib import Path

import torch
import torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]          # model/
sys.path.insert(0, str(ROOT))
from detect.net import DetectNet                    # noqa: E402
from classify.net import ClassifyNet                # noqa: E402


def op_of(m):
    if isinstance(m, nn.Conv2d):
        kind = 'depthwise-conv' if m.groups == m.in_channels else 'conv'
        return f'{kind} {m.kernel_size[0]}x{m.kernel_size[1]} s{m.stride[0]} p{m.padding[0]} g{m.groups}'
    if isinstance(m, nn.BatchNorm2d):
        return 'BN(推理时折进前面的 conv)'
    if isinstance(m, nn.ReLU):
        return 'ReLU'
    return type(m).__name__


def dump(net, size, title):
    rows, total_macs, total_params = [], 0, 0
    handles = []

    def hook(m, inp, out):
        nonlocal total_macs
        p = sum(x.numel() for x in m.parameters())
        macs = 0
        if isinstance(m, nn.Conv2d):
            oc, icg, kh, kw = m.weight.shape
            n, h, w = inp[0].shape[0], out.shape[-2], out.shape[-1]
            macs = int(n * h * w * kh * kw * icg * oc)
        total_macs += macs
        rows.append({
            'layer': m.__class__.__name__, 'op': op_of(m),
            'in': tuple(inp[0].shape[1:]), 'out': tuple(out.shape[1:]),
            'macs': macs, 'params': p})

    for m in net.modules():
        if isinstance(m, (nn.Conv2d, nn.BatchNorm2d, nn.ReLU)):
            handles.append(m.register_forward_hook(hook))
    net.eval()
    with torch.no_grad():
        net(torch.zeros(1, 3, size, size))
    for h in handles:
        h.remove()
    total_params = sum(p.numel() for p in net.parameters())

    out = [f'### {title}', '',
           '| # | 算子 | 输入 shape | 输出 shape | MACs | 参数 |', '|---|---|---|---|---|---|']
    for i, r in enumerate(rows, 1):
        out.append(f"| {i} | {r['op']} | {r['in']} | {r['out']} | {r['macs']:,} | {r['params']:,} |")
    out += ['', f"**合计**: MACs **{total_macs/1e6:.2f}M** @ {size}×{size}，参数 "
                f"**{total_params/1e3:.1f}K**", '']
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--markdown', action='store_true', help='输出 markdown 表格')
    args = ap.parse_args()
    txt = [dump(DetectNet('softargmax'), 144, 'A. DetectNet（手掌中心检测，输入 1×3×144×144）'),
           dump(ClassifyNet(), 80, 'B. ClassifyNet（三分类 张开/闭合/其它，输入 1×3×80×80）')]
    body = '\n'.join(txt)
    if args.markdown:
        print(body)
    else:
        print(body)


if __name__ == '__main__':
    main()
