"""三分类训练: CE(3 类: fist 闭合 / palm 张开 / other 其它)。

用法: python train.py [--epochs 40] [--batch 128] [--out ../frozen/classify_fp32.pt]
产物: 权重 + classify_train_history.json + classify_train_meta.json(数据来源与类别分布)
"""
import argparse
import json
import platform
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from net import ClassifyNet, count_macs, NUM_CLASSES
from dataset import ClassifyDataset, CLASSES, IMG_ROOT, NEG_EXTRA_ROOT

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--epochs', type=int, default=40)
    ap.add_argument('--batch', type=int, default=128)
    ap.add_argument('--lr', type=float, default=3e-3)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--no-recorded-other', action='store_true',
                    help='不把自录负样本帧(L1/机械臂)当 other 类')
    ap.add_argument('--out', type=Path, default=ROOT / 'frozen' / 'classify_fp32.pt')
    args = ap.parse_args()
    torch.manual_seed(0)
    np.random.seed(0)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    train_ds = ClassifyDataset('train', augment=True,
                               include_recorded=not args.no_recorded_other)
    val_ds = ClassifyDataset('val', augment=False,
                             include_recorded=not args.no_recorded_other)
    train_dl = torch.utils.data.DataLoader(
        train_ds, batch_size=args.batch, shuffle=True,
        num_workers=args.workers, persistent_workers=args.workers > 0, drop_last=True)
    val_dl = torch.utils.data.DataLoader(
        val_ds, batch_size=args.batch, shuffle=False,
        num_workers=args.workers, persistent_workers=args.workers > 0)

    net = ClassifyNet(NUM_CLASSES).to(device)
    opt = torch.optim.AdamW(net.parameters(), lr=args.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=args.epochs)
    ce = nn.CrossEntropyLoss()

    tr_cls, tr_src = train_ds.counts()
    va_cls, va_src = val_ds.counts()
    meta = {
        'classes': list(CLASSES), 'n_classes': NUM_CLASSES,
        'epochs': args.epochs, 'batch': args.batch, 'lr': args.lr, 'workers': args.workers,
        'img_root': str(IMG_ROOT),
        'recorded_other_root': str(NEG_EXTRA_ROOT.relative_to(ROOT)),
        'include_recorded_other': not args.no_recorded_other,
        'n_train': len(train_ds), 'n_val': len(val_ds),
        'train_by_class': tr_cls, 'train_by_source': tr_src,
        'val_by_class': va_cls, 'val_by_source': va_src,
        'skip_no_bbox_train': train_ds.skip_no_bbox, 'skip_no_bbox_val': val_ds.skip_no_bbox,
        'macs_M@80': round(count_macs(ClassifyNet(NUM_CLASSES)), 3),
        'params_K': round(sum(p.numel() for p in net.parameters()) / 1e3, 1),
        'device': device, 'torch': torch.__version__, 'python': platform.python_version(),
        'started_at': time.strftime('%Y-%m-%d %H:%M:%S'),
    }
    print(json.dumps(meta, indent=1, ensure_ascii=False), flush=True)

    best = {'acc': 0.0}
    args.out.parent.mkdir(parents=True, exist_ok=True)
    history = []
    t0 = time.time()
    for epoch in range(args.epochs):
        net.train()
        tot, n = 0.0, 0
        for x, y in train_dl:
            x, y = x.to(device), y.to(device)
            loss = ce(net(x), y)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tot += loss.item(); n += 1
        sched.step()
        net.eval()
        correct = total = 0
        with torch.no_grad():
            for x, y in val_dl:
                pred = net(x.to(device)).argmax(1).cpu()
                correct += (pred == y).sum().item()
                total += len(y)
        acc = correct / max(1, total)
        row = {'epoch': epoch + 1, 'loss': tot / n, 'val_acc': acc,
               'elapsed_sec': round(time.time() - t0, 1)}
        history.append(row)
        print(f"ep {epoch+1:02d} loss {row['loss']:.4f} | val acc {acc*100:.2f}%"
              f" [{row['elapsed_sec']:.0f}s]", flush=True)
        if acc > best['acc']:
            best['acc'] = acc
            torch.save(net.state_dict(), args.out)
            print(f'  -> 保存 {args.out}', flush=True)
    (args.out.parent / 'classify_train_history.json').write_text(
        json.dumps(history, indent=1), encoding='utf-8')
    meta.update({'finished_at': time.strftime('%Y-%m-%d %H:%M:%S'),
                 'total_sec': round(time.time() - t0, 1),
                 'best_epoch': int(np.argmax([r['val_acc'] for r in history]) + 1),
                 'best_val_acc': best['acc']})
    (args.out.parent / 'classify_train_meta.json').write_text(
        json.dumps(meta, indent=1, ensure_ascii=False), encoding='utf-8')
    print('完成, best val acc = %.2f%%' % (best['acc'] * 100))


if __name__ == '__main__':
    main()
