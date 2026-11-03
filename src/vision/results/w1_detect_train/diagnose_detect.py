"""检测网诊断：为什么 val 中位误差 ~26px（≈18% 画面）？只读分析，不改模型。

跑法（在 model/ 下，用项目环境）:
    /d/Anaconda3/envs/fpga_amd-pi/python.exe ../results/w1_detect_train/diagnose_detect.py

产出:
  - 控制台 JSON 报告
  - results/w1_detect_train/detect_pred_grid.png  (12 张 val 样本: 绿=GT 中心, 红=预测)
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

MODEL = Path(__file__).resolve().parents[2] / 'model'
sys.path.insert(0, str(MODEL))
sys.path.insert(0, str(MODEL / 'detect'))
from detect.net import DetectNet                     # noqa: E402
from detect.dataset import DetectDataset             # noqa: E402
from common.transforms import IMG_MEAN, IMG_STD      # noqa: E402

OUT = Path(__file__).resolve().parent
SIZE = 144


def collect(net, split, limit=None):
    """按序取正样本的 (pred_uv, gt_uv, x, conf, bbox 面积)。augment=False, 顺序与数据集一致。"""
    device = next(net.parameters()).device
    ds = DetectDataset(split, augment=False)
    n = len(ds)
    preds, gts, imgs, confs, areas = [], [], [], [], []
    net.eval()
    with torch.no_grad():
        for i in range(min(n, limit or n)):
            x, y = ds[i]
            if y[0] <= 0.5:
                continue
            p = net.predict(torch.from_numpy(x).unsqueeze(0).to(device))[0].cpu().numpy()
            bx, by, bw, bh = ds.pos[i][1]['bbox']          # 归一化 bbox
            preds.append(p[1:]); gts.append(y[1:]); imgs.append(x); confs.append(float(p[0]))
            areas.append(float(bw * bh))
    return np.array(preds), np.array(gts), np.array(imgs), np.array(confs), np.array(areas)


def feat_gap(net, x):
    """取 GAP 后的 96 维特征（conf 头之前）。"""
    feats = []

    def hook(_m, inp):
        feats.append(inp[0].detach())

    h = net.conf_head.register_forward_pre_hook(hook)
    with torch.no_grad():
        net(x)
    h.remove()
    return feats[0].squeeze()          # (96,)


def ridge(X, Y, lam=1.0):
    """闭式岭回归: (X^T X + lam I)^-1 X^T Y，X 已含偏置列。"""
    A = X.T @ X + lam * np.eye(X.shape[1])
    return np.linalg.solve(A, X.T @ Y)


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = DetectNet().to(device)
    w = Path(sys.argv[1]) if len(sys.argv) > 1 else MODEL / 'frozen' / 'detect_fp32.pt'
    net.load_state_dict(torch.load(w, map_location=device))
    print('权重:', w)
    try:
        wname = str(w.resolve().relative_to(MODEL.resolve()))
    except ValueError:
        wname = str(w)

    preds, gts, imgs, confs, areas = collect(net, 'val')
    err = np.hypot(preds[:, 0] - gts[:, 0], preds[:, 1] - gts[:, 1]) * SIZE
    rep = {
        'weights': wname,
        'n_pos': int(len(err)),
        'conf_recall@0.5': float((confs > 0.5).mean()),
        'pred_median_px': float(np.median(err)),
        'pred_p90_px': float(np.percentile(err, 90)),
        'err_percentiles_px': {k: round(float(np.percentile(err, k)), 2)
                               for k in (50, 75, 90, 95, 99, 100)},
        'frac_err_gt_8px': float((err > 8).mean()),
        'frac_err_gt_20px': float((err > 20).mean()),
        'frac_err_gt_40px': float((err > 40).mean()),
        'gt_std_px': [float(gts[:, 0].std() * SIZE), float(gts[:, 1].std() * SIZE)],
        'pearson_r': [float(np.corrcoef(preds[:, 0], gts[:, 0])[0, 1]),
                      float(np.corrcoef(preds[:, 1], gts[:, 1])[0, 1])],
        'pred_std_px': [float(preds[:, 0].std() * SIZE), float(preds[:, 1].std() * SIZE)],
    }
    # 尾部失败是否与手势尺寸相关?（小目标 = 手离镜头远/画面小）
    ok, bad = err <= 8, err > 40
    rep['tail_analysis'] = {
        'bbox_area_median_ok(<=8px)': round(float(np.median(areas[ok])), 5) if ok.any() else None,
        'bbox_area_median_bad(>40px)': round(float(np.median(areas[bad])), 5) if bad.any() else None,
        'n_ok': int(ok.sum()), 'n_bad': int(bad.sum()),
        'conf_median_ok': round(float(np.median(confs[ok])), 4) if ok.any() else None,
        'conf_median_bad': round(float(np.median(confs[bad])), 4) if bad.any() else None,
    }

    # ---- 关键：GT 中心是否落在 144x144 中心裁剪之内（出界 = 目标不可达，标签管线 bug）----
    oob = (gts[:, 0] < 0) | (gts[:, 0] > 1) | (gts[:, 1] < 0) | (gts[:, 1] > 1)
    inb = ~oob

    def stats(m):
        if not m.any():
            return None
        e = err[m]
        return {'n': int(m.sum()), 'median_px': round(float(np.median(e)), 2),
                'p90_px': round(float(np.percentile(e, 90)), 2),
                'frac_gt_8px': round(float((e > 8).mean()), 3)}

    rep['gt_out_of_crop'] = {
        'frac': round(float(oob.mean()), 4),
        'oob_stats': stats(oob), 'in_crop_stats': stats(inb),
        'share_of_gt8px_that_is_oob': round(float(oob[err > 8].mean()), 3) if (err > 8).any() else None,
    }
    lin = np.sqrt(areas)
    rep['size_conditioned'] = {
        f'hand_linear_size>={t}': stats(inb & (lin >= t)) for t in (0.0, 0.10, 0.15, 0.20)
    }

    # 基线1: 恒预测均值中心
    m = gts.mean(axis=0)
    base = np.hypot(gts[:, 0] - m[0], gts[:, 1] - m[1]) * SIZE
    rep['baseline_mean_center_median_px'] = float(np.median(base))
    # 基线2: 线性校准(把预测线性映射回 GT)后残差 —— 检验"有信息但被压缩/偏移"
    A = np.c_[preds, np.ones(len(preds))]
    coef = np.linalg.lstsq(A, gts, rcond=None)[0]
    cal = np.hypot(*(A @ coef - gts).T) * SIZE
    rep['calibrated_linear_median_px'] = float(np.median(cal))
    rep['calibration_coef'] = coef.tolist()

    # 线性探针: 冻结 backbone, 看 96 维 GAP 特征能否线性读出位置(与训练头对比)
    tr = DetectDataset('train', augment=False)
    idx = range(0, min(len(tr), 2500))
    Xtr, Ytr, Xva, Yva = [], [], [], []
    with torch.no_grad():
        for k, i in enumerate(idx):
            x, y = tr[i]
            if y[0] <= 0.5:
                continue
            f = feat_gap(net, torch.from_numpy(x).unsqueeze(0).to(device)).cpu().numpy()
            (Xtr if k % 5 else Xva).append(f)
            (Ytr if k % 5 else Yva).append(y[1:])
    Xtr, Ytr = np.array(Xtr), np.array(Ytr)
    Xva, Yva = np.array(Xva), np.array(Yva)
    Xtr = np.c_[Xtr, np.ones(len(Xtr))]; Xva = np.c_[Xva, np.ones(len(Xva))]
    coefp = ridge(Xtr, Ytr, lam=1.0)
    perr = np.hypot(*(Xva @ coefp - Yva).T) * SIZE
    rep['linear_probe_on_GAP_median_px'] = float(np.median(perr))
    rep['linear_probe_n_train_val'] = [int(len(Xtr)), int(len(Xva))]

    print(json.dumps(rep, indent=1, ensure_ascii=False))
    (OUT / 'detect_diagnose.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')

    # 可视化: 绿=GT, 红=预测。第一组=前 12 张, 第二组=误差最大的 12 张
    def save_grid(sel, path, label):
        tiles = []
        for i in sel:
            img = (imgs[i].transpose(1, 2, 0) * IMG_STD + IMG_MEAN) * 255
            img = np.clip(img, 0, 255).astype(np.uint8).copy()
            for pt, col in ((gts[i], (0, 255, 0)), (preds[i], (255, 0, 0))):
                cv2.drawMarker(img, (int(pt[0] * SIZE), int(pt[1] * SIZE)), col,
                               cv2.MARKER_CROSS, 9, 2)
            cv2.putText(img, f'{err[i]:.0f}px', (3, 138), cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                        (255, 255, 255), 1)
            tiles.append(img)
        while len(tiles) % 4:
            tiles.append(np.zeros_like(tiles[0]))
        rows = [np.hstack(tiles[r * 4:(r + 1) * 4]) for r in range(len(tiles) // 4)]
        grid = np.vstack(rows)
        cv2.putText(grid, label, (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1)
        cv2.imwrite(str(path), cv2.cvtColor(grid, cv2.COLOR_RGB2BGR))
        print('可视化:', path)

    save_grid(range(12), OUT / 'detect_pred_grid.png', 'first12  green=GT  red=pred')
    save_grid(list(np.argsort(-err)[:12]), OUT / 'detect_pred_grid_worst.png',
              'worst12 by error  green=GT  red=pred')


if __name__ == '__main__':
    main()
