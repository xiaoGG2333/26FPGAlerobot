"""手部检测训练: BCE(conf logit) + SmoothL1(u,v, 仅正样本)。

用法: python train.py [--epochs 40] [--batch 64] [--out ../frozen/detect_fp32.pt]

数据源: 默认 model/data/；设 HAGRID_ROOT 可切到镜像缓存（方案 A）:
    set HAGRID_ROOT=data/cache256
"""
import argparse
import itertools
import json
import platform
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from net import DetectNet, count_macs
from dataset import DetectDataset, IMG_ROOT, NEG_EXTRA_ROOT
from common.qfake import prepare_int8

ROOT = Path(__file__).resolve().parents[1]


def gauss_target(uv, h, w, sigma, device):
    """归一化中心 -> (B,1,h,w) 高斯热图目标(峰值 1)。格中心约定与 soft_argmax 一致。"""
    b = uv.size(0)
    cx = (uv[:, 0] * w - 0.5).view(b, 1, 1, 1)
    cy = (uv[:, 1] * h - 0.5).view(b, 1, 1, 1)
    yy = torch.arange(h, dtype=uv.dtype, device=device).view(1, 1, h, 1)
    xx = torch.arange(w, dtype=uv.dtype, device=device).view(1, 1, 1, w)
    d2 = (xx - cx) ** 2 + (yy - cy) ** 2
    t = torch.exp(-d2 / (2 * sigma ** 2))
    # 峰值归一化到 1（中心落在格边界时峰值不足 1，避免监督信号忽大忽小）
    return t / t.amax(dim=(1, 2, 3), keepdim=True).clamp_min(1e-6)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--epochs', type=int, default=40)
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--lr', type=float, default=3e-3)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--uv-weight', type=float, default=2.0)
    ap.add_argument('--uv-beta', type=float, default=0.02,
                    help='SmoothL1 的 beta；小值≈L1(尾部梯度更强，适合 ≤8px 紧门槛)')
    ap.add_argument('--hm-weight', type=float, default=0.0,
                    help='高斯热图辅助监督权重（0=关）；正样本 S4 热图 36x36')
    ap.add_argument('--hm-sigma', type=float, default=1.0, help='高斯热图 sigma（格）')
    ap.add_argument('--hm-loss', default='mse', choices=['mse', 'bce'])
    ap.add_argument('--head', default='softargmax', choices=['softargmax', 'gap'])
    ap.add_argument('--oob-neg-ratio', type=float, default=0.0,
                    help='"目标手在窗口外"负样本数 / 正样本数（0=关闭）')
    ap.add_argument('--zoom-min', type=float, default=0.88,
                    help='训练裁剪窗口缩放下限 k=size*U[zoom-min,1]；<0.88 开启多尺度增强（0.88=历史行为）')
    ap.add_argument('--neg-extra-weight', type=int, default=5,
                    help='自录域内负样本(L1/机械臂)在负样本池里的重复次数；0=不用')
    ap.add_argument('--qat-from', type=Path, default=None,
                    help='从该 FP32 权重出发做 INT8 QAT 微调（假量化+STE，lr 建议 3e-4）')
    ap.add_argument('--qat-mode', default='sym', choices=['sym', 'affine'])
    ap.add_argument('--qat-calib', type=int, default=16, help='QAT 校准批数（用 val 前 N 批）')
    ap.add_argument('--out', type=Path, default=ROOT / 'frozen' / 'detect_fp32.pt')
    args = ap.parse_args()
    torch.manual_seed(0)
    np.random.seed(0)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    train_ds = DetectDataset('train', augment=True, neg_extra_weight=args.neg_extra_weight,
                             oob_neg_ratio=args.oob_neg_ratio, zoom_min=args.zoom_min)
    val_ds = DetectDataset('val', augment=False)
    train_dl = torch.utils.data.DataLoader(
        train_ds, batch_size=args.batch, shuffle=True,
        num_workers=args.workers, persistent_workers=args.workers > 0, drop_last=True)
    val_dl = torch.utils.data.DataLoader(
        val_ds, batch_size=args.batch, shuffle=False,
        num_workers=args.workers, persistent_workers=args.workers > 0)

    net = DetectNet(args.head).to(device)
    qat_rep = None
    if args.qat_from:
        net.load_state_dict(torch.load(args.qat_from, map_location='cpu'))
        calib = [x for x, _ in itertools.islice(val_dl, args.qat_calib)]
        net, qat_rep = prepare_int8(net.cpu(), calib, mode=args.qat_mode,
                                    also_quantize=('uv_head.2',), ste=True, verbose=False)
        net = net.to(device)
        print('[QAT] 假量化已挂载 mode=%s 校准 %d 批；可训练参数 %d 个'
              % (args.qat_mode, len(calib), sum(p.numel() for p in net.parameters())), flush=True)
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    bce = nn.BCEWithLogitsLoss()
    l1 = nn.SmoothL1Loss(beta=args.uv_beta, reduction='none')

    meta = {
        'head': args.head, 'epochs': args.epochs, 'batch': args.batch, 'lr': args.lr,
        'workers': args.workers, 'uv_weight': args.uv_weight, 'uv_beta': args.uv_beta,
        'hm_weight': args.hm_weight, 'hm_sigma': args.hm_sigma, 'hm_loss': args.hm_loss,
        'keep_center': True,
        'img_root': str(IMG_ROOT),
        'zoom_min': args.zoom_min,
        'neg_extra': {
            'n': len(train_ds.neg_extra), 'weight': train_ds.neg_extra_weight,
            'root': str(NEG_EXTRA_ROOT.relative_to(ROOT)),
            'clips': sorted({p.parent.name for p in train_ds.neg_extra}),
            'n_hagrid_no_gesture': len(train_ds.neg),
            'neg_pool': len(train_ds.neg_pool),
        },
        'oob_neg': {'ratio': args.oob_neg_ratio, 'n_eligible': len(train_ds.pos_oob),
                    'n_slots': int(len(train_ds.pos_oob) * args.oob_neg_ratio)},
        'n_train': len(train_ds), 'n_val': len(val_ds),
        'macs_M@144': round(count_macs(DetectNet(args.head)), 2),
        'params_K': round(sum(p.numel() for p in net.parameters()) / 1e3, 1),
        'qat': ({'from': str(args.qat_from), 'mode': args.qat_mode,
                 'calib_batches': args.qat_calib, 'ste': True,
                 'act_snr_min': min(v['snr_db'] for v in qat_rep['layers'].values())}
                if qat_rep else None),
        'device': device, 'torch': torch.__version__, 'python': platform.python_version(),
        'started_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    print(json.dumps(meta, indent=1, ensure_ascii=False), flush=True)

    best = {'err_px': 1e9}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    history = []
    t0 = time.time()
    for epoch in range(args.epochs):
        net.train()
        tot_loss, tot_conf, tot_uv, tot_hm, n = 0.0, 0.0, 0.0, 0.0, 0
        for x, y in train_dl:
            x, y = x.to(device), y.to(device)
            conf_logit, uv, hm = net(x, return_hm=True) if args.hm_weight > 0 else (*net(x), None)
            conf_loss = bce(conf_logit, y[:, 0])
            mask = (y[:, 0] > 0.5).float()
            uv_loss = (l1(uv, y[:, 1:]) * mask.unsqueeze(1)).sum() / max(1.0, mask.sum())
            loss = conf_loss + args.uv_weight * uv_loss
            if hm is not None:
                tgt = gauss_target(y[:, 1:], hm.size(2), hm.size(3), args.hm_sigma, hm.device)
                if args.hm_loss == 'mse':
                    hm_loss = ((torch.sigmoid(hm) - tgt) ** 2).mean(dim=(1, 2, 3))
                else:
                    hm_loss = nn.functional.binary_cross_entropy_with_logits(
                        hm, tgt, reduction='none').mean(dim=(1, 2, 3))
                hm_loss = (hm_loss * mask).sum() / max(1.0, mask.sum())
                loss = loss + args.hm_weight * hm_loss
                tot_hm += hm_loss.item()
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot_loss += loss.item(); tot_conf += conf_loss.item(); tot_uv += uv_loss.item(); n += 1
        sched.step()
        # ---- 验证 ----
        net.eval()
        preds, gts = [], []
        with torch.no_grad():
            for x, y in val_dl:
                p = net.predict(x.to(device)).cpu().numpy()
                y = y.numpy()
                for i in range(len(y)):
                    if y[i, 0] > 0.5 and p[i, 0] > 0.5:
                        preds.append(p[i, 1:]); gts.append(y[i, 1:])
        err = np.array([np.hypot(a[0] - b[0], a[1] - b[1]) * 144 for a, b in zip(preds, gts)])
        err_px = float(np.median(err)) if len(err) else 1e9
        n_pos = len(err)
        row = {'epoch': epoch + 1, 'loss': tot_loss / n, 'conf': tot_conf / n,
               'uv': tot_uv / n, 'hm': tot_hm / n, 'val_median_px': err_px, 'val_n': n_pos,
               'elapsed_sec': round(time.time() - t0, 1)}
        history.append(row)
        print(f"ep {epoch+1:02d} loss {row['loss']:.4f} conf {row['conf']:.4f} uv {row['uv']:.4f}"
              f" | val median {err_px:.2f}px (n={n_pos}) [{row['elapsed_sec']/60:.1f}min]", flush=True)
        if err_px < best['err_px']:
            best['err_px'] = err_px
            best['epoch'] = epoch + 1
            torch.save(net.state_dict(), args.out)
            print(f'  -> 保存 {args.out}', flush=True)
    (args.out.parent / 'detect_train_history.json').write_text(json.dumps(history, indent=1), encoding='utf-8')
    meta.update({'finished_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                 'total_sec': round(time.time() - t0, 1),
                 'best_epoch': best.get('epoch', -1), 'best_val_median_px': best['err_px']})
    meta['oob_neg'].update(train_ds.oob_stats)          # 实际生效/回退条数
    (args.out.parent / 'detect_train_meta.json').write_text(
        json.dumps(meta, indent=1, ensure_ascii=False), encoding='utf-8')
    print('完成, best val median = %.2f px (epoch %d)' % (best['err_px'], best.get('epoch', -1)))


if __name__ == '__main__':
    main()
