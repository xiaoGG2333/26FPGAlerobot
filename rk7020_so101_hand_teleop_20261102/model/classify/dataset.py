"""分类数据加载 (v2: 三分类 张开/闭合/其它)。

标签语义 (主人 2026-10-14 定义):
    0 = fist  五指闭合(握拳)        → 夹爪闭合
    1 = palm  五指张开              → 夹爪张开
    2 = other 其它一切(竖拇指/OK/剪刀手/自然搭手/无手背景) → **夹爪不动**

数据来源:
    hagrid_subset/dataset.json 里 group 字段: closed / open / other
        closed = fist      open = palm
        other  = like(竖拇指) + ok + peace(剪刀手) + no_gesture(自然搭手)
    自录负样本帧(recorded_neg/frames/*): L1 环境空镜 + 机械臂静止 → group=other,
        框取画面中心 0.5 短边(模拟检测误触发的 ROI); 2/3 进 train, 1/3 进 val

ROI: 按 bbox 裁剪(随机 1.3~1.8x 扩展) → 80x80, 与检测框中心扩展语义一致。
"""
import json
import os
import random

import cv2
import numpy as np
from pathlib import Path
from torch.utils.data import Dataset

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.transforms import photometric, motion_blur, normalize

ROOT = Path(__file__).resolve().parents[1]
DATA_JSON = ROOT / 'data' / 'hagrid_subset' / 'dataset.json'
NEG_EXTRA_ROOT = ROOT / 'data' / 'recorded_neg' / 'frames'

CLASSES = ('fist', 'palm', 'other')          # 索引即标签
CLS_OF_GROUP = {'closed': 0, 'open': 1, 'other': 2}


def _img_root():
    """图片根目录。默认 data/；设 HAGRID_ROOT 可切到镜像缓存(如 data/cache256)。"""
    env = os.environ.get('HAGRID_ROOT', '').strip()
    if not env:
        return ROOT / 'data'
    p = Path(env)
    return p if p.is_absolute() else (ROOT / p)


IMG_ROOT = _img_root()
if not IMG_ROOT.is_dir():
    raise FileNotFoundError(f'图片根目录不存在: {IMG_ROOT}（HAGRID_ROOT 设置是否正确？）')


def _list_neg_extra():
    if not NEG_EXTRA_ROOT.is_dir():
        return []
    return sorted(p for p in NEG_EXTRA_ROOT.glob('*/*.jpg') if p.is_file())


def crop_roi(img, bbox, expand, rng, size=80, augment=True):
    """按 bbox(归一化) 裁剪 ROI -> size×size。augment 时扩展系数与中心点带抖动。"""
    h, w = img.shape[:2]
    bx, by, bw, bh = bbox
    if augment:
        ex = expand * rng.uniform(0.9, 1.25)
        jx = rng.uniform(-0.12, 0.12)
        jy = rng.uniform(-0.12, 0.12)
    else:
        ex, jx, jy = expand, 0.0, 0.0
    cw, ch = bw * w * ex, bh * h * ex
    cx, cy = (bx + bw / 2 + jx * bw) * w, (by + bh / 2 + jy * bh) * h
    x0, y0 = int(cx - cw / 2), int(cy - ch / 2)
    x1, y1 = int(cx + cw / 2), int(cy + ch / 2)
    # 越界处理: 平移回界内; 完全出界则回退到全图
    if x1 - x0 < 8 or y1 - y0 < 8:
        return cv2.resize(img, (size, size), interpolation=cv2.INTER_AREA)
    pad_x, pad_y = max(0, -x0), max(0, -y0)
    x0, x1 = max(0, x0), min(w, x1)
    y0, y1 = max(0, y0), min(h, y1)
    roi = img[y0:y1, x0:x1]
    if pad_x or pad_y:
        roi = cv2.copyMakeBorder(roi, pad_y, 0, pad_x, 0, cv2.BORDER_REFLECT)
    roi = cv2.resize(roi, (size, size), interpolation=cv2.INTER_AREA)
    if augment and rng.random() < 0.5:
        roi = roi[:, ::-1]
    return roi


class ClassifyDataset(Dataset):
    """split 同时作用于 HaGRID 部分(标注自带 train/val/test)与自录负样本(按帧序 2:1)。"""

    def __init__(self, split='train', size=80, augment=True, seed=0, include_recorded=True):
        self.size, self.augment = size, augment
        data = json.loads(DATA_JSON.read_text(encoding='utf-8'))[split]
        self.items, self.skip_no_bbox = [], 0
        for uuid, v in data.items():
            grp = v.get('group')
            if grp is None:      # 兼容 v1 清单
                grp = {'fist': 'closed', 'palm': 'open'}.get(v['label'], 'other')
            if grp not in CLS_OF_GROUP:
                continue
            if v.get('bbox') is None:      # 无框无法裁 ROI(仅 no_gesture 可能)
                self.skip_no_bbox += 1
                continue
            self.items.append({'uuid': uuid, 'path': IMG_ROOT / v['img'], 'bbox': v['bbox'],
                               'cls': CLS_OF_GROUP[grp], 'source': v['label']})
        self.n_hagrid = len(self.items)
        if include_recorded:
            for i, p in enumerate(_list_neg_extra()):
                if (i % 3 == 2) == (split == 'val'):       # 2/3 train, 1/3 val
                    # 中心 0.5 短边的方框 ≈ 检测误触发时给出的 ROI 位置
                    self.items.append({'uuid': p.stem, 'path': p,
                                       'bbox': [0.25, 0.25, 0.5, 0.5], 'cls': 2,
                                       'source': 'recorded_bg'})
        self.rng = random.Random(seed)

    def __len__(self):
        return len(self.items)

    def counts(self):
        """{class_index: n} 与 {source: n}，用于训练元信息与汇总。"""
        by_cls = {i: 0 for i in range(len(CLASSES))}
        by_src = {}
        for it in self.items:
            by_cls[it['cls']] += 1
            by_src[it['source']] = by_src.get(it['source'], 0) + 1
        return {CLASSES[k]: v for k, v in by_cls.items()}, by_src

    def __getitem__(self, idx):
        it = self.items[idx]
        img = cv2.imread(str(it['path']))
        if img is None:
            raise ValueError(f"无法读取 {it['path']}")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        roi = crop_roi(img, it['bbox'], 1.5, self.rng, self.size, self.augment)
        if self.augment:
            if self.rng.random() < 0.5:
                roi = photometric(roi, self.rng)
            if self.rng.random() < 0.4:
                roi = motion_blur(roi, self.rng)
        x = normalize(roi).transpose(2, 0, 1).astype(np.float32)
        return x, np.array(it['cls'], dtype=np.int64)
