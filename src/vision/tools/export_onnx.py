# -*- coding: utf-8 -*-
"""把 train_kpt.py 存下的 best.pt 导出成 ONNX（train-pi 环境）。

只做导出，不做评估。IO 与 live_kpt.py / eval_kpt.py 保持一致：
  input  img      [1,3,S,S] float32（S 来自 config 里的 size）
  output coord    [1,42]  ROI 归一化 21x2
         presence [1]     **logit**（训练用 BCEWithLogits，用前要 sigmoid）
         handed   [1,2]   Left/Right logits

--logits：导另一种头，热图 logits 直接出图，softmax/soft-argmax 放到图外：
  output hm       [1,21,hm,hm] 热图 logits（hm=S/stride）
         presence [1]
         handed   [1,2]
  这是上硬件/量化时应该用的形式：图内 exp 是 int8 最脆的部位。

用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/export_onnx.py \
      --ckpt results/stage1/self_run2/best.pt --out results/stage1/self_run2/best.onnx
  ... --logits  -> 多出一个 *_hm.onnx
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from train_kpt import HandKptNet, HeatmapHead  # noqa: E402


class _LogitsWrap(nn.Module):
    """热图 logits 出图：把 softmax + 质心解码留在图外（量化/硬件友好的形式）。"""

    def __init__(self, net):
        super().__init__()
        self.net = net

    def forward(self, x):
        _, p, h, logits = self.net.forward_train(x)
        return logits, p, h


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ckpt', required=True)
    ap.add_argument('--out', default=None)
    ap.add_argument('--logits', action='store_true', help='额外导热图 logits 版本')
    args = ap.parse_args()

    ck = torch.load(args.ckpt, map_location='cpu', weights_only=False)
    cfg = ck.get('config', {})
    ins = int(cfg.get('size', 256))
    net = (HeatmapHead(stride=cfg.get('hm_stride', 16), sigma=cfg.get('sigma', 1.2), in_size=ins)
           if cfg.get('head') == 'heatmap' else HandKptNet(in_size=ins))
    net.load_state_dict(ck['model'])
    net.eval()
    out = Path(args.out) if args.out else Path(args.ckpt).with_suffix('.onnx')
    torch.onnx.export(net, torch.zeros(1, 3, ins, ins), str(out),
                      input_names=['img'], output_names=['coord', 'presence', 'handed'],
                      dynamo=False, opset_version=17)
    print(f'[导出] {out}  {out.stat().st_size/1e6:.2f} MB  input {ins}x{ins}  epoch={ck.get("epoch")} '
          f'val_nme={ck.get("val_nme")}', flush=True)
    if args.logits:
        if cfg.get('head') != 'heatmap':
            print('[跳过] 只有 heatmap 头能导 logits 版', flush=True)
            return
        w = _LogitsWrap(net).eval()
        out2 = out.with_name(out.stem + '_hm.onnx')
        torch.onnx.export(w, torch.zeros(1, 3, ins, ins), str(out2),
                          input_names=['img'], output_names=['hm', 'presence', 'handed'],
                          dynamo=False, opset_version=17)
        print(f'[导出] {out2}  {out2.stat().st_size/1e6:.2f} MB  hm={ins // int(cfg.get("hm_stride", 16))}',
              flush=True)


if __name__ == '__main__':
    main()
