"""「目标手在窗口外」负样本的可行性与形态探针（只读）。

回答三个问题：
  1. 有多少比例的正样本真能造出"中心被推出窗口"的窗口（按图像纵横比分类）？
  2. 推出的距离分布如何（归一化 / 144px 尺度的像素）？
  3. 现冻结检测网在这些窗口上有多自信（= 要压制的误触发）？并出图目视确认。
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

MODEL = Path(__file__).resolve().parents[2] / 'model'
sys.path.insert(0, str(MODEL))
sys.path.insert(0, str(MODEL / 'detect'))
from common.transforms import geom_oob, map_point, normalize, bbox_center_px   # noqa: E402
from detect.dataset import DetectDataset, IMG_ROOT                             # noqa: E402
from detect.net import DetectNet                                               # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=800)
    ap.add_argument('--weights', type=Path, default=MODEL / 'frozen' / 'detect_fp32.pt')
    ap.add_argument('--out-dir', type=Path,
                    default=Path(__file__).resolve().parent)
    ap.add_argument('--device', default='cuda' if torch.cuda.is_available() else 'cpu')
    args = ap.parse_args()

    import random
    ds = DetectDataset('train', augment=True)
    rng = random.Random(7)
    idx = list(range(len(ds.pos)))
    rng.shuffle(idx)
    idx = idx[:args.n]

    feas, dlt, aspect = [], [], {'portrait': [0, 0], 'landscape': [0, 0], 'square': [0, 0]}
    keep = []                      # 出图用的样本
    for i in idx:
        uuid, v = ds.pos[i]
        path = IMG_ROOT / v['img']
        h, w = cv2.imread(str(path)).shape[:2]
        bu, bv = bbox_center_px(v['bbox'], 1.0, 1.0)
        key = 'portrait' if h > w else ('landscape' if w > h else 'square')
        aspect[key][1] += 1
        g = geom_oob(h, w, 144, random.Random(i), (bu, bv))
        if g is None:
            continue
        aspect[key][0] += 1
        u, vv = map_point(bu, bv, g)
        d = float(np.hypot(max(0.0, u - 1.0) + max(0.0, -u), max(0.0, vv - 1.0) + max(0.0, -vv)))
        feas.append((i, v, u, vv))
        dlt.append(d)
        if len(keep) < 12:
            keep.append((i, v, u, vv, h, w))

    dlt = np.array(dlt)
    rep = {
        'n_sampled': len(idx), 'n_feasible': len(feas),
        'feasible_rate': round(len(feas) / max(1, len(idx)), 4),
        'by_aspect': {k: {'feasible': a, 'total': b, 'rate': round(a / max(1, b), 4)}
                      for k, (a, b) in aspect.items()},
        'out_norm': {'min': round(float(dlt.min()), 4), 'median': round(float(np.median(dlt)), 4),
                     'max': round(float(dlt.max()), 4)},
        'out_px@144': {'min': round(float(dlt.min()) * 144, 1),
                       'median': round(float(np.median(dlt)) * 144, 1),
                       'max': round(float(dlt.max()) * 144, 1)},
        'tiles': [],
    }

    # 冻结网在这些窗口上的 conf（= 要压制的误触发强度）
    net = DetectNet().to(args.device).eval()
    net.load_state_dict(torch.load(args.weights, map_location=args.device))
    imgs, meta = [], []
    for i, v, u, vv, h, w in keep:
        path = IMG_ROOT / v['img']
        img = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
        sc = 144 / min(h, w)
        img = cv2.resize(img, (int(round(w * sc)), int(round(h * sc))), interpolation=cv2.INTER_AREA)
        g = geom_oob(h, w, 144, random.Random(i), bbox_center_px(v['bbox'], 1.0, 1.0))
        y0, x0, ch, cw = g[2]
        crop = img[y0:y0 + ch, x0:x0 + cw]
        crop = cv2.resize(crop, (144, 144), interpolation=cv2.INTER_AREA)   # k<144 时放大到 144 便于比对
        imgs.append(crop)
        meta.append((u, vv))
    x = torch.from_numpy(np.stack([normalize(c).transpose(2, 0, 1) for c in imgs]).astype(np.float32))
    with torch.no_grad():
        out = net(x.to(args.device))
        conf = torch.sigmoid(out[0]).cpu().numpy().ravel() if isinstance(out, tuple) else None
        uv = out[1].cpu().numpy() if isinstance(out, tuple) else None
    tiles = []
    for k, (im, (u, vv)) in enumerate(zip(imgs, meta)):
        t = cv2.resize(im, (144, 144))
        px = int(round(np.clip(uv[k, 0], 0, 1) * 144)), int(round(np.clip(uv[k, 1], 0, 1) * 144))
        cv2.circle(t, px, 5, (255, 0, 0), 2)                       # 蓝=模型预测点
        cv2.putText(t, f'c={conf[k]:.2f} out={np.hypot(max(0,u-1)+max(0,-u), max(0,vv-1)+max(0,-vv))*144:.0f}px',
                    (2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1)
        cv2.rectangle(t, (0, 0), (143, 143), (200, 200, 200), 1)
        tiles.append(t)
        rep['tiles'].append({'conf': round(float(conf[k]), 4),
                             'pred_uv': [round(float(uv[k, 0]), 3), round(float(uv[k, 1]), 3)],
                             'gt_out_px': round(float(np.hypot(max(0, u - 1) + max(0, -u),
                                                               max(0, vv - 1) + max(0, -vv)) * 144), 1)})
    rep['frozen_conf_on_oob'] = {
        'median': round(float(np.median(conf)), 4), 'max': round(float(conf.max()), 4),
        'fire@0.5': round(float(np.mean(conf > 0.5)), 4), 'n': len(tiles)}
    sheet = np.concatenate([np.concatenate(tiles[r:r + 4], axis=1) for r in range(0, 4 * (len(tiles) // 4), 4)], axis=0)
    cv2.imwrite(str(args.out_dir / 'oob_samples.png'), cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))
    (args.out_dir / 'oob_probe.json').write_text(json.dumps(rep, indent=1, ensure_ascii=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in rep.items() if k != 'tiles'}, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
