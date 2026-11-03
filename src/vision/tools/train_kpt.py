# -*- coding: utf-8 -*-
"""T4：sanity 模型训练（train-pi 环境）。

结构（决策①=A）：MobileNetV2 width 0.75、去最后 stage（cut=14，特征 16x16@256²）、纯 ReLU6，
GAP+FC 三头：21x2 坐标(sigmoid) + presence(1) + handedness(2)。
损失：5.0*Wing(w=10,eps=2，仅 presence=1 帧) + 1.0*BCE(presence) + 0.5*CE(handedness，仅正样本)。
用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/train_kpt.py --cache results/stage1/cache_xxx --epochs 1    # 冒烟
  D:/Anaconda3/envs/train-pi/python.exe -u tools/train_kpt.py --cache results/stage1/cache_xxx --epochs 40   # 长训
产物：results/stage1/sanity_run1/{config.json, best.pt, last.pt, history.json}
"""
import argparse
import json
import math
import random
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, Dataset

SEED = 42


def set_seed(s=SEED):
    random.seed(s)
    np.random.seed(s)
    torch.manual_seed(s)
    torch.cuda.manual_seed_all(s)


def wing_loss(pred, tgt, w=10.0, eps=2.0):
    d = (pred - tgt).abs()
    C = w * math.log(1.0 + w / eps)
    return torch.where(d < w, w * torch.log1p(d / eps), d - C)


class HeatmapHead(nn.Module):
    """stride-8  trunk + 21 通道 32x32 热图 + 网内 soft-argmax → 坐标。

    导出 ONNX 后输出仍是 21x2 坐标（soft-argmax 在图内）；部署端若要把热图交后端算，
    用踩坑清单 §F 的归一化 ReLU 质心（零 exp）替代 softmax。
    """

    def __init__(self, width_mult=0.75, hid=256, hm=32, sigma=1.2, in_size=256, stride=8):
        super().__init__()
        from torchvision.models import mobilenet_v2
        feats = list(mobilenet_v2(weights=None, width_mult=width_mult).features)[:14]
        if stride == 8:
            # 首层保持 s2（浅层 128²，算力可控）；把 IR24 首块的 depthwise(ConvBNReLU, idx1) s2 改 s1 → 总步幅 8
            feats[2].conv[1][0].stride = (1, 1)   # ConvBNReLU 是 Sequential，stride 在内层 conv
            hm = in_size // 8
        else:
            hm = in_size // 16
        self.feat = nn.Sequential(*feats)
        with torch.no_grad():
            f0 = self.feat(torch.zeros(1, 3, in_size, in_size))
            ch = f0.shape[1]
            assert f0.shape[2] == f0.shape[3] == hm, f'stride 错配: feat {tuple(f0.shape)} vs hm {hm}'
        self.hm_conv = nn.Conv2d(ch, 21, 1)
        self.hm = hm
        self.sigma = sigma
        gy, gx = torch.meshgrid(torch.arange(hm, dtype=torch.float32) + 0.5,
                                torch.arange(hm, dtype=torch.float32) + 0.5,
                                indexing='ij')
        self.register_buffer('gx', gx / hm)
        self.register_buffer('gy', gy / hm)
        self.pool = nn.Sequential(nn.AdaptiveAvgPool2d(1), nn.Flatten(),
                                  nn.Linear(ch, hid), nn.ReLU6())
        self.pres = nn.Linear(hid, 1)
        self.hand = nn.Linear(hid, 2)

    def heat_targets(self, y):                        # y: (B,42) -> (B,21,hm,hm)
        c = (y.view(-1, 21, 2) * self.hm).clamp(0, self.hm)
        dx = c[..., 0][:, :, None, None] - self.gx[None, None] * self.hm
        dy = c[..., 1][:, :, None, None] - self.gy[None, None] * self.hm
        return torch.exp(-(dx * dx + dy * dy) / (2 * self.sigma ** 2))

    def forward_train(self, x):
        f = self.feat(x)
        logits = self.hm_conv(f)                      # (B,21,hm,hm)
        B = logits.shape[0]
        flat = logits.flatten(2)
        w = torch.softmax(flat, dim=-1).view(B, 21, self.hm, self.hm)
        cx = (w * self.gx[None, None]).sum(dim=(2, 3))
        cy = (w * self.gy[None, None]).sum(dim=(2, 3))
        coord = torch.stack([cx, cy], dim=-1).flatten(1)
        g = self.pool(f)
        return coord, self.pres(g).squeeze(-1), self.hand(g), logits

    def forward(self, x):
        c, p, h, _ = self.forward_train(x)
        return c, p, h


class HandKptNet(nn.Module):
    def __init__(self, width_mult=0.75, cut=14, hid=256, in_size=256):
        super().__init__()
        from torchvision.models import mobilenet_v2
        feats = list(mobilenet_v2(weights=None, width_mult=width_mult).features)[:cut]
        self.feat = nn.Sequential(*feats)
        with torch.no_grad():
            ch = self.feat(torch.zeros(1, 3, in_size, in_size)).shape[1]
        self.pool = nn.Sequential(nn.AdaptiveAvgPool2d(1), nn.Flatten(),
                                  nn.Linear(ch, hid), nn.ReLU6())
        self.coord = nn.Linear(hid, 42)
        self.pres = nn.Linear(hid, 1)
        self.hand = nn.Linear(hid, 2)

    def forward(self, x):
        f = self.pool(self.feat(x))
        return torch.sigmoid(self.coord(f)), self.pres(f).squeeze(-1), self.hand(f)


def count_mac(net, size=256):
    total = [0]
    hooks = []

    def hk(mod, inp, outp):
        if isinstance(mod, nn.Conv2d):
            total[0] += int(np.prod(outp.shape[2:])) * mod.in_channels // mod.groups \
                * mod.out_channels * mod.kernel_size[0] * mod.kernel_size[1]
        elif isinstance(mod, nn.Linear):
            total[0] += mod.in_features * mod.out_features
    for m in net.modules():
        if isinstance(m, (nn.Conv2d, nn.Linear)):
            hooks.append(m.register_forward_hook(hk))
    dev = next(net.parameters()).device
    with torch.no_grad():
        net(torch.zeros(1, 3, size, size, device=dev))
    for h in hooks:
        h.remove()
    return total[0]


class CacheDS(Dataset):
    """懒加载 mmap：memmap 对象被 pickle 进 worker 时会整块进内存（Windows spawn 必炸），
    故只存路径，在每个进程首次 __getitem__ 时各自 open。"""

    def __init__(self, cache, idx, aug=False, size=256):
        self.p = Path(cache)
        self.idx = list(idx)
        self.aug = aug
        self.size = int(size)
        self._m = None

    def _open(self):
        if self._m is None:
            self._m = {k: np.load(self.p / f'{k}.npy', mmap_mode='r')
                       for k in ('imgs', 'pts', 'presence', 'hand', 'scale')}

    def __len__(self):
        return len(self.idx)

    def __getitem__(self, k):
        self._open()
        i = self.idx[k]
        img = np.asarray(self._m['imgs'][i], np.uint8)
        if img.shape[0] != self.size:                 # 缓存是 256²，按 --size 重采样
            img = cv2.resize(img, (self.size, self.size), interpolation=cv2.INTER_AREA)
        y = np.asarray(self._m['pts'][i], np.float32).reshape(42).copy()
        if self.aug:
            sz = img.shape[0]
            ang = random.uniform(-15, 15)
            M = cv2.getRotationMatrix2D((sz / 2, sz / 2), ang, 1.0)
            img = cv2.warpAffine(img, M, (sz, sz), borderMode=cv2.BORDER_REPLICATE)
            P = y.reshape(21, 2) * sz
            P = (M @ np.hstack([P, np.ones((21, 1))]).T).T / sz
            y = np.clip(P, 0, 1).reshape(42).astype(np.float32)
            if random.random() < 0.5:
                a = random.uniform(0.8, 1.2)
                b = random.uniform(-20, 20)
                img = np.clip(img.astype(np.float32) * a + b, 0, 255).astype(np.uint8)
            if random.random() < 0.3:
                img = cv2.blur(img, (3, 3))
        x = (img.astype(np.float32).transpose(2, 0, 1) / 127.5) - 1.0
        return (torch.from_numpy(x),
                torch.from_numpy(y),
                torch.tensor(float(self._m['presence'][i])),
                torch.tensor(int(self._m['hand'][i])),
                torch.tensor(float(self._m['scale'][i])))


def nme_of(pred, tgt, scale):
    """pred/tgt: (N,21,2) 归一化；scale: (N,) 教师手长。返回平均 NME。"""
    err = np.linalg.norm(pred - tgt, axis=2)              # (N,21)
    return float((err.mean(axis=1) / np.maximum(scale, 1e-6)).mean())


def evaluate(net, ds, loader, device):
    net.eval()
    P, T, S = [], [], []
    with torch.no_grad():
        for x, y, pr, hd, sc in loader:
            c, _, _ = net(x.to(device))
            P.append(c.cpu().numpy())
            T.append(y.numpy())
            S.append(sc.numpy())
    P = np.concatenate(P).reshape(-1, 21, 2)
    T = np.concatenate(T).reshape(-1, 21, 2)
    return nme_of(P, T, np.concatenate(S))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cache', required=True)
    ap.add_argument('--out', default='results/stage1/sanity_run1')
    ap.add_argument('--epochs', type=int, default=40)
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--warmup', type=int, default=5)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--head', choices=['gap', 'heatmap'], default='gap')
    ap.add_argument('--hm-stride', type=int, choices=[8, 16], default=8)
    ap.add_argument('--sigma', type=float, default=1.2)
    ap.add_argument('--aug', action='store_true', help='旋转±15°/亮度对比/运动模糊')
    ap.add_argument('--size', type=int, default=256, help='输入边长（缓存 256²，会重采样）')
    args = ap.parse_args()

    set_seed()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    split = json.loads((Path(args.cache) / 'split.json').read_text())
    train_idx = split['train'] + split.get('neg', [])
    random.shuffle(train_idx)

    ds_tr = CacheDS(args.cache, train_idx, aug=args.aug, size=args.size)
    ds_va = CacheDS(args.cache, split['val'], size=args.size)
    dl_tr = DataLoader(ds_tr, batch_size=args.batch, shuffle=True,
                       num_workers=args.workers, pin_memory=True,
                       persistent_workers=args.workers > 0, drop_last=True)
    dl_va = DataLoader(ds_va, batch_size=128, shuffle=False, num_workers=2)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    net = (HeatmapHead(stride=args.hm_stride, sigma=args.sigma, in_size=args.size) if args.head == 'heatmap'
           else HandKptNet(in_size=args.size)).to(device)
    n_par = sum(p.numel() for p in net.parameters())
    n_mac = count_mac(net, args.size)
    print(f'[模型] 参数 {n_par/1e6:.2f}M  MACs@{args.size}² {n_mac/1e6:.1f}M  设备 {device}', flush=True)

    opt = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=1e-4)
    total_steps = max(1, len(dl_tr)) * args.epochs
    warm_steps = max(1, len(dl_tr)) * args.warmup
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt, lambda s: (s / warm_steps) if s < warm_steps else
        0.5 * (1 + math.cos(math.pi * (s - warm_steps) / max(1, total_steps - warm_steps))))
    scaler = torch.amp.GradScaler('cuda', enabled=device.type == 'cuda')
    bce = nn.BCEWithLogitsLoss()
    ce = nn.CrossEntropyLoss()

    history = []
    best = 1e9
    t0 = time.time()
    for ep in range(args.epochs):
        net.train()
        losses = []
        for x, y, pr, hd, sc in dl_tr:
            x = x.to(device, non_blocking=True)
            y = y.to(device)
            pr = pr.to(device)
            hd = hd.to(device)
            opt.zero_grad(set_to_none=True)
            with torch.amp.autocast('cuda', enabled=device.type == 'cuda'):
                if args.head == 'heatmap':
                    c, p_logit, h_logit, logits = net.forward_train(x)
                else:
                    c, p_logit, h_logit = net(x)
                m = pr > 0.5
                l_coord = wing_loss(c[m], y[m]).mean() if m.any() else c.sum() * 0.0
                l_pres = bce(p_logit, pr)
                mh = m & (hd >= 1)                      # hand_code 1/2 -> 类别 0/1
                l_hand = ce(h_logit[mh], hd[mh] - 1) if mh.any() else h_logit.sum() * 0.0
                loss = 5.0 * l_coord + 1.0 * l_pres + 0.5 * l_hand
                if args.head == 'heatmap':
                    tgt_hm = net.heat_targets(y)
                    loss = loss + 0.5 * nn.functional.binary_cross_entropy_with_logits(
                        logits[m], tgt_hm[m])
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            sched.step()
            losses.append(float(loss))
        val_nme = evaluate(net, ds_va, dl_va, device)
        hist = {'epoch': ep + 1, 'loss': round(float(np.mean(losses)), 5),
                'val_nme': round(val_nme, 5),
                'lr': round(opt.param_groups[0]['lr'], 6),
                'sec': round(time.time() - t0, 1)}
        history.append(hist)
        print(f"[ep {ep+1:3d}] loss={hist['loss']:.4f} val_nme={val_nme:.4f} "
              f"lr={hist['lr']:.2e} t={hist['sec']:.0f}s", flush=True)
        if val_nme < best:
            best = val_nme
            torch.save({'model': net.state_dict(), 'epoch': ep + 1,
                        'val_nme': val_nme, 'config': vars(args),
                        'n_params': n_par, 'n_macs': n_mac}, out / 'best.pt')
        if (ep + 1) % 5 == 0 or ep == args.epochs - 1:
            torch.save({'model': net.state_dict(), 'epoch': ep + 1}, out / 'last.pt')
            (out / 'history.json').write_text(json.dumps(history, indent=1),
                                              encoding='utf-8')

    cfg = dict(vars(args))
    cfg.update({'seed': SEED, 'n_params': n_par, 'n_macs': n_mac,
                'best_val_nme': round(best, 5),
                'created': time.strftime('%Y-%m-%d %H:%M:%S'),
                'torch': torch.__version__, 'cuda': torch.version.cuda})
    (out / 'config.json').write_text(json.dumps(cfg, ensure_ascii=False, indent=1),
                                     encoding='utf-8')
    (out / 'history.json').write_text(json.dumps(history, indent=1), encoding='utf-8')
    print(f'[完成] best_val_nme={best:.4f}  产物在 {out}', flush=True)


if __name__ == '__main__':
    main()
