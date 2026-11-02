"""标注噪声探测：HaGRID 的 bbox 标注有多可靠？（决定尾部 8–40px 是否可达）

两个实验（都用**部署同款**裁剪 augment=False）：
  A. 训练集误差 vs 验证集误差 —— 若两者接近，尾部不是"过拟合/泛化"问题。
  B. 近似重复配对 —— 同一 user_id 且画面几乎一致的 val 样本对：
       模型预测差(Δpred) vs 标注差(Δgt)。若 Δgt 明显大于 Δpred，说明**标注本身抖**，
       即模型的一致性已经优于标签，8–40px 的尾部不可能靠模型修掉。

用法:
    /d/Anaconda3/envs/fpga_amd-pi/python.exe ../results/w1_detect_train/check_label_noise.py [权重路径]
"""
import json
import sys
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
MODEL = HERE.parents[1] / 'model'
sys.path.insert(0, str(MODEL))
from detect.net import DetectNet                    # noqa: E402
from detect.dataset import DetectDataset, DATA_JSON  # noqa: E402

SIZE = 144


def collect(split, net, device):
    ds = DetectDataset(split, augment=False)
    meta = json.loads(DATA_JSON.read_text(encoding='utf-8'))[split]
    preds, gts, imgs, users, keys = [], [], [], [], []
    net.eval()
    with torch.no_grad():
        for i in range(len(ds)):
            x, y = ds[i]
            if y[0] <= 0.5:
                continue
            uuid = ds.pos[i][0]
            p = net.predict(torch.from_numpy(x).unsqueeze(0).to(device))[0].cpu().numpy()
            preds.append(p[1:]); gts.append(y[1:]); imgs.append(x)
            users.append(meta[uuid]['user_id']); keys.append(uuid)
    return (np.array(preds), np.array(gts), np.array(imgs), np.array(users), keys)


def err_px(preds, gts):
    return np.hypot(preds[:, 0] - gts[:, 0], preds[:, 1] - gts[:, 1]) * SIZE


def main():
    wp = Path(sys.argv[1]) if len(sys.argv) > 1 else MODEL / 'frozen' / 'detect_fp32.pt'
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = DetectNet().to(device)
    net.load_state_dict(torch.load(wp, map_location=device))
    print('权重:', wp)

    rep = {'weights': str(wp)}
    # --- A: train vs val ---
    for split in ('train', 'val'):
        p, g, im, us, ks = collect(split, net, device)
        e = err_px(p, g)
        rep[f'{split}_error'] = {'n': int(len(e)), 'median_px': round(float(np.median(e)), 2),
                                 'p90_px': round(float(np.percentile(e, 90)), 2),
                                 'frac_gt_8px': round(float((e > 8).mean()), 3)}
        if split == 'val':
            vp, vg, vi, vus = p, g, im, us
    print(json.dumps({k: v for k, v in rep.items() if k.endswith('_error')}, indent=1))

    # --- B: 近似重复配对 ---
    pairs = []
    for i in range(len(vus)):
        for j in range(i + 1, len(vus)):
            if vus[i] != vus[j]:
                continue
            a, b = vi[i], vi[j]
            diff = float(np.abs(a[:, ::4, ::4] - b[:, ::4, ::4]).mean())
            dgt = float(np.hypot(*(vg[i] - vg[j])) * SIZE)
            dpred = float(np.hypot(*(vp[i] - vp[j])) * SIZE)
            pairs.append((diff, dgt, dpred))
    pairs.sort()                                    # 按画面差异从小到大
    arr = np.array(pairs)
    k = min(300, max(1, len(arr) // 10))
    closest = arr[:k]
    rep['dup_pairs'] = {
        'n_pairs_same_user': int(len(arr)),
        'n_closest': int(k),
        'img_diff_median_closest': round(float(np.median(closest[:, 0])), 2),
        'gt_diff_px_median_closest': round(float(np.median(closest[:, 1])), 2),
        'pred_diff_px_median_closest': round(float(np.median(closest[:, 2])), 2),
        'gt_diff_px_p90_closest': round(float(np.percentile(closest[:, 1], 90)), 2),
        'pred_diff_px_p90_closest': round(float(np.percentile(closest[:, 2], 90)), 2),
        'share_closest_where_gt_jumps_gt8px': round(float((closest[:, 1] > 8).mean()), 3),
        'share_closest_where_pred_jumps_gt8px': round(float((closest[:, 2] > 8).mean()), 3),
    }
    print(json.dumps(rep['dup_pairs'], indent=1))
    (HERE / 'detect_label_noise.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print('已写入', HERE / 'detect_label_noise.json')


if __name__ == '__main__':
    main()
