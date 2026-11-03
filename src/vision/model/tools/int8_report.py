"""INT8 量化报告：逐层 scale 表 + 逐层误差预算表（供 HLS / 《网络结构清单》§5 用）。

用法（工程根）:
    cd model && HAGRID_ROOT=data/cache256 python ../model/tools/int8_report.py --mode sym

输出:
    results/w1_int8/int8_layer_table.md   （人看：逐层 scale + SNR）
    results/w1_int8/int8_layer_table_<mode>.json（程序读：完整量化表）
"""
import argparse
import json
import sys
from pathlib import Path

import torch

ROOT = Path(__file__).resolve().parents[1]          # model/
PROJ = ROOT.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(PROJ / 'model' / 'detect'))
sys.path.insert(0, str(PROJ / 'model' / 'classify'))
sys.path.insert(0, str(PROJ / 'model' / 'common'))

from common.qfake import prepare_int8, layer_snr, conv_layers, fold_bn, _stats_of  # noqa: E402


def detect_batches(n, device, bs=32):
    from detect.dataset import DetectDataset
    dl = torch.utils.data.DataLoader(DetectDataset('val', augment=False), batch_size=bs, shuffle=False)
    out = []
    for i, (x, _) in enumerate(dl):
        out.append(x.to(device))
        if (i + 1) * bs >= n:
            break
    return out


def classify_batches(n, device, bs=64):
    from classify.dataset import ClassifyDataset
    dl = torch.utils.data.DataLoader(ClassifyDataset('val', augment=False), batch_size=bs, shuffle=False)
    out = []
    for i, (x, _) in enumerate(dl):
        out.append(x.to(device))
        if (i + 1) * bs >= n:
            break
    return out


def weight_health(net):
    """逐层权重健康度：量化后被压成 0 的比例、通道内动态范围（HLS 设计参考）。"""
    out = {}
    for name, m in conv_layers(net):
        w = m.weight.data.reshape(m.weight.shape[0], -1)
        amax = w.abs().amax(dim=1, keepdim=True).clamp_min(1e-8)
        s = amax / 127
        out[name] = {
            'w_dead_frac': round(float((w.abs() / s < 0.5).float().mean()), 5),
            'ch_dyn_range_median': round(float((amax.squeeze() / w.abs().clamp_min(1e-8).amin(dim=1)).median()), 2),
        }
    return out


def run(net_fn, batches, mode, name, also_quantize):
    net_fp = fold_bn(net_fn())          # 对照基准必须也是「BN 已折叠」的浮点网
    net_q, rep = prepare_int8(net_fp, batches, mode=mode, also_quantize=also_quantize)
    snr = layer_snr(net_fp, net_q, batches[0])
    return rep, snr, weight_health(net_fp)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode', choices=['sym', 'affine'], default='sym')
    ap.add_argument('--n', type=int, default=128, help='校准样本数（报告用，正式评测用全 val）')
    ap.add_argument('--out-dir', type=Path, default=PROJ / 'results' / 'w1_int8')
    args = ap.parse_args()
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    args.out_dir.mkdir(parents=True, exist_ok=True)

    from detect.net import DetectNet
    from classify.net import ClassifyNet
    jobs = [
        ('DetectNet', lambda: DetectNet().to(device).eval(),
         'frozen/detect_fp32.pt', detect_batches, ('uv_head.2',)),
        ('ClassifyNet', lambda: ClassifyNet().to(device).eval(),
         'frozen/classify_fp32.pt', classify_batches, ()),
    ]
    full, md = {}, []
    for name, fn, wpath, gen, also in jobs:
        net = fn()
        net.load_state_dict(torch.load(ROOT / wpath, map_location=device))
        batches = gen(args.n, device)
        rep, snr, wh = run(lambda: net, batches, args.mode, name, also)
        full[name] = {'weights': wpath, 'quant': rep, 'layer_output_snr': snr, 'weight_health': wh}
        md.append(f'\n### {name}（校准 {rep["calib_samples"]} 张，mode={args.mode}）\n')
        md.append('| 层 | 输入 scale `s_x` | zp | 权重 scale `s_w`(中位/最大) | 权重压成0比例 | 输入量化 SNR | 输出量化 SNR | 输出余弦 | 校准 clip 比例 |')
        md.append('|---|---|---|---|---|---|---|---|---|')
        for ln, v in rep['layers'].items():
            o = snr.get(ln, {})
            md.append(f"| `{ln}` | {v['s']:.6g} | {int(v['zp'])} | "
                      f"{v['s_w_mean']:.4g} / {v['s_w_max']:.4g} | {wh[ln]['w_dead_frac']*100:.2f}% | {v['snr_db']:.1f} dB | "
                      f"{o.get('out_snr_db', float('nan')):.1f} dB | {o.get('cos', float('nan')):.5f} | "
                      f"{v['clip_frac']*100:.3f}% |")
        md.append('')
        print(f'[{name}] 输入 SNR 最低 {min(v["snr_db"] for v in rep["layers"].values()):.1f} dB | '
              f'输出 SNR 最低 {min(o["out_snr_db"] for o in snr.values()):.1f} dB')

    head = (f'# INT8 量化逐层表（PTQ，mode={args.mode}，{args.n} 张校准样本）\n\n'
            f'口径：BN 折叠进 conv；权重逐输出通道对称；激活逐张量（{args.mode}）；'
            f'int32 精确累加；softmax/sigmoid 浮点。\n'
            f'**输出 SNR** = 该层输出相对浮点输出的信噪比（逐层误差预算，HLS 用）。')
    txt = head + '\n'.join(md)
    (args.out_dir / 'int8_layer_table.md').write_text(txt, encoding='utf-8')
    (args.out_dir / f'int8_layer_table_{args.mode}.json').write_text(
        json.dumps(full, indent=1), encoding='utf-8')
    print('已写入', args.out_dir / 'int8_layer_table.md',
          '与', args.out_dir / f'int8_layer_table_{args.mode}.json')


if __name__ == '__main__':
    main()
