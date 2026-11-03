"""闭环自检兜底的「开销 + 有效性」实测。

问题 1：兜底能不能用？（conf 已经证明不区分成败：坏样本 conf 0.991）
  - 热图熵 H(p)：坏样本的热图是否更"散"？
  - 时序一致性：近同帧之间预测是否"跳"？
问题 2：兜底要花多少？（算成 PL 周期 / A9 时间 / 相对检测网 MACs）

跑法（在 model/ 下）:
    D:/Anaconda3/envs/fpga_amd-pi/python.exe ../results/w1_detect_train/check_selfcheck.py [权重]
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
MODEL = HERE.parents[1] / 'model'
sys.path.insert(0, str(MODEL))
from detect.net import DetectNet                     # noqa: E402
from detect.dataset import DetectDataset, DATA_JSON  # noqa: E402

SIZE = 144
CLK_MHZ = 100          # PL 目标时钟（ROADMAP M1.8）
FRAME_PL_MS = 33.0     # PL 版 30fps 预算
FRAME_A9_MS = 66.0     # A9 保命版 15fps 预算
DETECT_MACS = 21.06e6
A9_OPS_PER_S = 1.0e8   # A9 纯 C 定点保守有效吞吐（量级估计）


def collect(net):
    """val 正样本：热图 prob、熵、峰值、误差、是否出框。"""
    device = next(net.parameters()).device
    ds = DetectDataset('val', augment=False)
    meta = json.loads(DATA_JSON.read_text(encoding='utf-8'))['val']
    ent, maxp, err, oob, preds, gts, imgs, users, confs = [], [], [], [], [], [], [], [], []
    ring, bpred = [], []
    net.eval()
    with torch.no_grad():
        for i in range(len(ds)):
            x, y = ds[i]
            if y[0] <= 0.5:
                continue
            xt = torch.from_numpy(x).unsqueeze(0).to(device)
            _c, uv, p = net.heatmap(xt)
            confs.append(float(_c[0]))
            p = p[0, 0].cpu().numpy().astype(np.float64)
            uv = uv[0].cpu().numpy()
            ent.append(float(-(p * np.log2(p + 1e-12)).sum()))
            maxp.append(float(p.max()))
            # 边界信号：预测点靠边 / 热图质量压在边缘环上（手在画面外时的应有反应）
            h, w = p.shape
            acc = p.reshape(-1)
            k = 0.05
            edge = ((np.arange(h)[:, None] < k * h) | (np.arange(h)[:, None] > (1 - k) * h) |
                    (np.arange(w)[None, :] < k * w) | (np.arange(w)[None, :] > (1 - k) * w))
            ring.append(float(acc[edge.reshape(-1)].sum()))
            bpred.append(float(min(uv[0], 1 - uv[0], uv[1], 1 - uv[1])))
            err.append(float(np.hypot(*(uv - y[1:])) * SIZE))
            oob.append(bool(y[1] < 0 or y[1] > 1 or y[2] < 0 or y[2] > 1))
            preds.append(uv); gts.append(y[1:]); imgs.append(x)
            users.append(meta[ds.pos[i][0]]['user_id'])
    return dict(ent=np.array(ent), maxp=np.array(maxp), err=np.array(err), oob=np.array(oob),
                preds=np.array(preds), gts=np.array(gts), imgs=np.array(imgs), users=np.array(users),
                conf=np.array(confs), ring=np.array(ring), bpred=np.array(bpred))


def disc(a, err, bad_thr=40.0):
    """a 作为异常分数（越大越可疑）时，区分 err>bad_thr 的能力。"""
    bad = err > bad_thr
    out = {'n_bad': int(bad.sum()), 'median_a_good': round(float(np.median(a[~bad])), 3),
           'median_a_bad': round(float(np.median(a[bad])), 3) if bad.any() else None}
    r = np.corrcoef(a, err)[0, 1]
    out['pearson_r_with_err'] = round(float(r), 3)
    flags = []
    for pct in (50, 70, 80, 90, 95):                    # 按分数分位设阈值 = 放行 (100-pct)%
        thr = float(np.percentile(a, pct))
        f = a > thr
        flags.append({'thr_pct': pct, 'flag_rate': round(float(f.mean()), 3),
                      'recall_bad': round(float((f & bad).sum() / max(1, bad.sum())), 3),
                      'precision_bad': round(float((f & bad).sum() / max(1, f.sum())), 3)})
    out['flag_table'] = flags
    return out


def temporal(d, tau_px):
    """近同帧配对（同 user_id、画面差异最小十分位）：预测跳变能否抓到坏样本。"""
    imgs, preds, err, users = d['imgs'], d['preds'], d['err'], d['users']
    pairs = []
    for i in range(len(users)):
        for j in range(i + 1, len(users)):
            if users[i] != users[j]:
                continue
            diff = float(np.abs(imgs[i][:, ::4, ::4] - imgs[j][:, ::4, ::4]).mean())
            dpred = float(np.hypot(*(preds[i] - preds[j])) * SIZE)
            pairs.append((diff, dpred, max(err[i], err[j]), min(err[i], err[j])))
    pairs.sort()
    k = max(1, len(pairs) // 10)
    close = np.array(pairs[:k])
    jump = close[:, 1] > tau_px
    bad = close[:, 2] > 40
    return {'n_pairs_close': int(k), 'img_diff_med': round(float(np.median(close[:, 0])), 2),
            'pred_jump_med_px': round(float(np.median(close[:, 1])), 2),
            'flag_rate': round(float(jump.mean()), 3),
            'recall_bad_endpoint': round(float((jump & bad).sum() / max(1, bad.sum())), 3),
            'precision_bad_endpoint': round(float((jump & bad).sum() / max(1, jump.sum())), 3)}


def cost():
    """静态开销核算。热图 36x36=1296 格。"""
    n = 36 * 36
    softmax_ops = 3 * n                      # exp + 累加 + 归一化（现有，已有）
    ent_ops = 3 * n                          # LUT log2 + mul + add
    maxp_ops = n                             # 比较
    t_pl_1cyc_us = n / CLK_MHZ               # 1 格/周期
    t_pl_4cyc_us = 4 * n / CLK_MHZ           # 4 周期/格
    return {
        'heatmap_cells': n,
        'existing_softmax_ops': softmax_ops,
        'entropy_extra_ops': ent_ops,
        'maxp_extra_ops': maxp_ops,
        'entropy_vs_detect_MACs_pct': round(100 * ent_ops / DETECT_MACS, 4),
        'entropy_pl_us_1cell_per_cycle': round(t_pl_1cyc_us, 1),
        'entropy_pl_us_4cycle_per_cell': round(t_pl_4cyc_us, 1),
        'entropy_pl_pct_of_33ms_budget': round(100 * t_pl_4cyc_us / 1000 / FRAME_PL_MS, 3),
        'entropy_a9_ms': round(ent_ops / A9_OPS_PER_S * 1000, 3),
        'entropy_a9_pct_of_66ms_budget': round(100 * (ent_ops / A9_OPS_PER_S * 1000) / FRAME_A9_MS, 3),
        'temporal_extra_ops': 10,
        'temporal_vs_detect_MACs_pct': round(100 * 10 / DETECT_MACS, 6),
        'temporal_latency_frames': 1,
        'ring_border_extra_ops': 2,          # 归一化坐标与 0.05 比一下
        'ring_border_vs_detect_MACs_pct': round(100 * 2 / DETECT_MACS, 8),
        'note': 'PL 侧 log2 用 LUT(256x16bit=512B BRAM) 或 CORDIC；时序一致性只需 4 个寄存器',
    }


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    net = DetectNet().to(device)
    w = Path(sys.argv[1]) if len(sys.argv) > 1 else MODEL / 'frozen' / 'detect_fp32.pt'
    net.load_state_dict(torch.load(w, map_location=device))
    print('权重:', w)
    t0 = time.time()
    d = collect(net)
    err = d['err']
    rep = {'weights': str(w), 'n_pos': int(len(err)),
           'err_median_px': round(float(np.median(err)), 2),
           'frac_bad_gt8px': round(float((err > 8).mean()), 3),
           'frac_bad_gt40px': round(float((err > 40).mean()), 3)}
    # 1) 有效性
    rep['entropy_score'] = disc(d['ent'], err)            # 熵大 = 散 = 可疑
    rep['inv_maxp_score'] = disc(-d['maxp'], err)         # 峰值低 = 可疑
    rep['inv_conf_score'] = disc(-d['conf'], err)         # 旧结论：不区分成败，重测确认
    rep['ring_mass_score'] = disc(d['ring'], err)          # 边缘质量占比大 = 可疑
    rep['border_pred_score'] = disc(-d['bpred'], err)      # 预测点离边越近越可疑
    # 专项：能不能抓到「手出画」这类熵也抓不到的失效
    oob = d['oob']
    rep['oob_specific'] = {
        'n_oob': int(oob.sum()),
        'entropy_recall_on_oob': round(float((d['ent'][oob] > np.percentile(d['ent'], 80)).mean()), 3),
        'ring_recall_on_oob': round(float((d['ring'][oob] > np.percentile(d['ring'], 80)).mean()), 3),
        'border_recall_on_oob': round(float((d['bpred'][oob] < np.percentile(d['bpred'], 20)).mean()), 3),
        'oob_ring_med': round(float(np.median(d['ring'][oob])), 3),
        'in_ring_med': round(float(np.median(d['ring'][~oob])), 3),
        'oob_bpred_med': round(float(np.median(d['bpred'][oob])), 3),
        'in_bpred_med': round(float(np.median(d['bpred'][~oob])), 3),
    }
    rep['oob_share'] = round(float(d['oob'].mean()), 3)
    rep['oob_entropy_med'] = round(float(np.median(d['ent'][d['oob']])), 3) if d['oob'].any() else None
    rep['in_entropy_med'] = round(float(np.median(d['ent'][~d['oob']])), 3)
    rep['temporal_tau8'] = temporal(d, 8.0)
    rep['temporal_tau20'] = temporal(d, 20.0)
    # 2) 开销
    rep['cost'] = cost()
    rep['host_seconds'] = round(time.time() - t0, 1)
    print(json.dumps(rep, indent=1, ensure_ascii=False))
    (HERE / 'detect_selfcheck.json').write_text(json.dumps(rep, indent=1, ensure_ascii=False),
                                                encoding='utf-8')
    print('已写入', HERE / 'detect_selfcheck.json')


if __name__ == '__main__':
    main()
