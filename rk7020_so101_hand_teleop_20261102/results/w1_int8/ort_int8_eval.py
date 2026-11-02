"""第三方交叉验证：用 onnxruntime 官方静态 INT8 量化（QDQ，per-channel）复测两模型精度。

目的：避免"自己写的模拟器偏乐观"。它不替代 `model/common/qfake.py`（那个才产出 HLS 要用的
scale 表、且口径可控），而是给同一组权重一个**独立实现**的量化结果做对照。

用法（工程根）:
    cd model && HAGRID_ROOT=data/cache256 python ../results/w1_int8/ort_int8_eval.py --which both
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]          # 工程根
sys.path.insert(0, str(ROOT / 'model'))
sys.path.insert(0, str(ROOT / 'model' / 'detect'))
sys.path.insert(0, str(ROOT / 'model' / 'classify'))
from common.qfake import fold_bn                     # noqa: E402


class Wrap(torch.nn.Module):
    """把 net 包成「返回单个张量」的图，便于 ONNX 导出。"""

    def __init__(self, net, kind):
        super().__init__()
        self.net, self.kind = net, kind

    def forward(self, x):
        if self.kind == 'detect':
            conf, uv = self.net(x)
            return torch.stack([conf, uv[:, 0], uv[:, 1]], dim=1)     # (N,3) = conf_logit,u,v
        return self.net(x)


def export_onnx(net, kind, size, path):
    net = fold_bn(net).eval()
    dummy = torch.zeros(1, 3, size, size)
    try:                        # 旧版 TorchScript 导出器（不依赖 onnxscript）
        torch.onnx.export(Wrap(net, kind), dummy, str(path), input_names=['x'],
                          output_names=['out'], opset_version=17, do_constant_folding=True,
                          dynamic_axes={'x': {0: 'batch'}, 'out': {0: 'batch'}}, dynamo=False)
    except TypeError:
        torch.onnx.export(Wrap(net, kind), dummy, str(path), input_names=['x'],
                          output_names=['out'], opset_version=17, do_constant_folding=True)
    return path


class Reader:
    def __init__(self, batches, name='x'):
        self.batches, self.name = list(batches), name
        self.i = 0

    def get_next(self):
        if self.i >= len(self.batches):
            return None
        b = self.batches[self.i][0:self.i + 1] if False else self.batches[self.i]
        self.i += 1
        return {self.name: np.ascontiguousarray(b.numpy().astype(np.float32))}

    def rewind(self):
        self.i = 0


def quantize(fp32_path, int8_path, batches, op_types=None):
    """op_types=None → 量化 ORT 认为可量化的全部算子；['Conv'] → 只量化卷积（对照：把 soft-argmax 算术留在浮点）。"""
    from onnxruntime.quantization import quantize_static, QuantFormat, QuantType, CalibrationMethod
    kw = {} if op_types is None else {'op_types_to_quantize': op_types}
    quantize_static(str(fp32_path), str(int8_path), Reader(batches),
                    quant_format=QuantFormat.QDQ, per_channel=True, activation_type=QuantType.QInt8,
                    weight_type=QuantType.QInt8, calibrate_method=CalibrationMethod.MinMax, **kw)
    return int8_path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--which', choices=['detect', 'classify', 'both'], default='both')
    ap.add_argument('--out-dir', type=Path, default=ROOT / 'results' / 'w1_int8')
    ap.add_argument('--jitter-n', type=int, default=30)
    ap.add_argument('--op-types', choices=['all', 'conv'], default='all',
                    help='all=量化 ORT 全部可量化算子；conv=只量化卷积（soft-argmax 算术留浮点）')
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    import onnxruntime as ort

    res = {}
    if args.which in ('detect', 'both'):
        from detect.net import DetectNet
        from detect.dataset import DetectDataset
        net = DetectNet().eval()
        net.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'detect_fp32.pt', map_location='cpu'))
        ds = DetectDataset('val', augment=False)
        dl = torch.utils.data.DataLoader(ds, batch_size=32, shuffle=False)
        batches = [x for x, _ in dl]
        onnx_fp = export_onnx(net, 'detect', 144, args.out_dir / 'detect_fp32.onnx')
        op_types = None if args.op_types == 'all' else ['Conv']
        suffix = '' if args.op_types == 'all' else '_convonly'
        onnx_i8 = quantize(onnx_fp, args.out_dir / f'detect_int8_ort{suffix}.onnx', batches, op_types)
        sess = ort.InferenceSession(str(onnx_i8), providers=['CPUExecutionProvider'])
        n_pos, n_hag = len(ds.pos), len(ds.neg)
        rec_clip = [p.parent.name for p in ds.neg_extra]
        pos_err, pos_conf, neg_conf, rec_conf = [], [], [], {}
        off = 0
        rng = np.random.default_rng(0)
        jit = []
        for x in batches:
            out = sess.run(None, {'x': x.numpy().astype(np.float32)})[0]
            conf = 1 / (1 + np.exp(-out[:, 0]))
            for i in range(len(x)):
                gi = off + i
                if gi < n_pos:
                    y = ds[gi][1].numpy() if hasattr(ds[gi][1], 'numpy') else np.array(ds[gi][1])
                    e = float(np.hypot(out[i, 1] - y[1], out[i, 2] - y[2]) * 144)
                    pos_err.append(e)
                    if conf[i] > 0.5:
                        pos_conf.append(e)
                    if args.jitter_n and y[0] > 0.5:
                        batch = []
                        for _ in range(args.jitter_n):
                            dx, dy = int(rng.integers(-2, 3)), int(rng.integers(-2, 3))
                            batch.append(np.roll(x[i].numpy(), (dy, dx), axis=(1, 2)))
                        o = sess.run(None, {'x': np.stack(batch)})[0]
                        pts = o[:, 1:] * 144
                        jit.append(float(np.mean(np.std(pts, axis=0))))
                else:
                    k = gi - n_pos
                    (rec_conf.setdefault(rec_clip[k - n_hag], []).append(float(conf[i]))
                     if k >= n_hag else neg_conf.append(float(conf[i])))
            off += len(x)
        rows = {k: {'n': len(v), 'conf_median': round(float(np.median(v)), 4),
                    'conf_max': round(float(max(v)), 4),
                    'fire@0.5': round(float(np.mean(np.array(v) > 0.5)), 4)}
                for k, v in list(rec_conf.items()) + [('hagrid_no_gesture', neg_conf)]}
        res['detect'] = {
            'onnx': str(onnx_i8.name), 'n_pos': n_pos,
            'median_px': round(float(np.median(pos_conf)), 3) if pos_conf else None,
            'p90_px': round(float(np.percentile(pos_conf, 90)), 2) if pos_conf else None,
            'n_accepted': len(pos_conf), 'recall@0.5': round(len(pos_conf) / max(1, n_pos), 4),
            'median_all_px': round(float(np.median(pos_err)), 3),
            'jitter_px': round(float(np.mean(jit)), 3) if jit else None,
            'negatives': rows,
        }
        print('[ORT detect]', json.dumps(res['detect'], ensure_ascii=False)[:400])

    if args.which in ('classify', 'both'):
        from classify.net import ClassifyNet, NUM_CLASSES
        from classify.dataset import ClassifyDataset, CLASSES
        net = ClassifyNet(NUM_CLASSES).eval()
        net.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'classify_fp32.pt', map_location='cpu'))
        ds = ClassifyDataset('val', augment=False)
        dl = torch.utils.data.DataLoader(ds, batch_size=64, shuffle=False)
        batches = [x for x, _ in dl]
        onnx_fp = export_onnx(net, 'classify', 80, args.out_dir / 'classify_fp32.onnx')
        op_types = None if args.op_types == 'all' else ['Conv']
        suffix = '' if args.op_types == 'all' else '_convonly'
        onnx_i8 = quantize(onnx_fp, args.out_dir / f'classify_int8_ort{suffix}.onnx', batches, op_types)
        sess = ort.InferenceSession(str(onnx_i8), providers=['CPUExecutionProvider'])
        n = len(CLASSES)
        cm = np.zeros((n, n), dtype=int)
        src_of = [it['source'] for it in ds.items]
        off = 0
        for x in batches:
            logits = sess.run(None, {'x': x.numpy().astype(np.float32)})[0]
            pred = logits.argmax(1)
            for i in range(len(x)):
                cm[ds.items[off + i]['cls'], pred[i]] += 1
            off += len(x)
        acc = float(cm.diagonal().sum() / cm.sum())
        other_n, other_move = int(cm[2].sum()), int(cm[2, 0] + cm[2, 1])
        res['classify'] = {
            'onnx': str(onnx_i8.name), 'val_acc': round(acc, 4),
            'per_class_recall': {CLASSES[k]: round(cm[k, k] / max(1, cm[k].sum()), 4) for k in range(n)},
            'other_motion_rate': round(other_move / max(1, other_n), 4),
            'confusion': {CLASSES[i]: {CLASSES[j]: int(cm[i, j]) for j in range(n)} for i in range(n)},
        }
        print('[ORT classify]', json.dumps(res['classify'], ensure_ascii=False))

    tag = 'all' if args.op_types == 'all' else 'convonly'
    out = args.out_dir / f'ort_int8_eval_{tag}.json'
    if out.exists():                      # 合并（--which 分次跑时不覆盖已有结果）
        merged = json.loads(out.read_text(encoding='utf-8'))
        merged.update(res)
        res = merged
    out.write_text(json.dumps(res, indent=1), encoding='utf-8')
    print('已写入', args.out_dir / f'ort_int8_eval_{tag}.json')


if __name__ == '__main__':
    main()
