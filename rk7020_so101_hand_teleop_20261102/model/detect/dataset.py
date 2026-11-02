"""检测数据加载: fist/palm 正样本(中心点标签) + no_gesture 负样本(conf=0)。

几何变换(裁剪/缩放/翻转)与中心点坐标同步映射。
"""
import json
import os
import random

import numpy as np
from pathlib import Path
from torch.utils.data import Dataset

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.transforms import (load_jpeg_geom, load_jpeg_oob, oob_feasible, image_size,
                               normalize, bbox_center_px, map_point, photometric)

ROOT = Path(__file__).resolve().parents[1]
DATA_JSON = ROOT / 'data' / 'hagrid_subset' / 'dataset.json'
NEG_EXTRA_ROOT = ROOT / 'data' / 'recorded_neg' / 'frames'


def _img_root():
    """图片根目录。默认 data/；设 HAGRID_ROOT 可切到镜像缓存(方案 A, 如 data/cache256)。

    相对路径按 model/ 解析；缓存保持与 data/ 相同的相对结构，
    因此 dataset.json 的 img 字段无需改动。
    """
    env = os.environ.get('HAGRID_ROOT', '').strip()
    if not env:
        return ROOT / 'data'
    p = Path(env)
    return p if p.is_absolute() else (ROOT / p)


IMG_ROOT = _img_root()
if not IMG_ROOT.is_dir():
    raise FileNotFoundError(f'图片根目录不存在: {IMG_ROOT}（HAGRID_ROOT 设置是否正确？）')


def _list_neg_extra():
    """自录域内负样本帧（主人拍的 L1 环境空镜 + 机械臂静止）——按 <片段>/*.jpg 收集。"""
    if not NEG_EXTRA_ROOT.is_dir():
        return []
    return sorted(p for p in NEG_EXTRA_ROOT.glob('*/*.jpg') if p.is_file())


class DetectDataset(Dataset):
    """size: 输入边长；augment=True 时裁剪窗口保证包含标注中心(keep_center)并做增强。

    neg_extra_weight: 自录域内负样本在负样本池中的重复次数（1 次=不额外加权）。
    oob_neg_ratio: 每个 epoch 的"目标手在窗口外"负样本条数 = **可用正样本数** × 该倍数
                   （可用 = 标注中心能被推出窗口的图，见 transforms.oob_feasible；仅训练集）。
    """

    def __init__(self, split='train', size=144, augment=True, neg_ratio=0.4, seed=0, zoom_min=0.88,
                 neg_extra_weight=5, oob_neg_ratio=0.0):
        self.size, self.augment = size, augment
        self.zoom_min = float(zoom_min)   # 多尺度增强: 训练裁剪窗口 k=size*U[zoom_min,1]（默认 0.88=历史行为）
        data = json.loads(DATA_JSON.read_text(encoding='utf-8'))[split]
        self.pos = [(uuid, v) for uuid, v in data.items() if v['label'] in ('fist', 'palm')]
        self.neg = [(uuid, v) for uuid, v in data.items() if v['label'] == 'no_gesture']
        self.neg_extra = _list_neg_extra() if neg_extra_weight > 0 else []
        self.neg_extra_weight = max(0, neg_extra_weight)
        # 训练负样本池 = HaGRID no_gesture 各一次 + 自录域内负样本重复 w 次
        self.neg_pool = ([('hagrid', i) for i in range(len(self.neg))] +
                         [('rec', i) for i in range(len(self.neg_extra))] * self.neg_extra_weight)
        # 必须确定性打乱：每个 epoch 只用池的前 int(len(pos)*neg_ratio) 条，
        # 若保持(hagrid 在前、自录在后)的顺序，自录负样本永远排在切片之外 → 一条都训不到
        # （2026-10-14 实测踩坑：1781 个槽位里自录条数 = 0）。
        random.Random(1234).shuffle(self.neg_pool)
        self.neg_ratio = neg_ratio if augment else 1.0
        self.oob_neg_ratio = oob_neg_ratio if augment else 0.0
        self.oob_stats = {'feasible': 0, 'fallback': 0}      # 实际生效比例（写进 meta）
        # 预筛"标注中心能被推出窗口"的正样本：HaGRID 标注中心多集中在画面中部，
        # 不预筛的话绝大多数槽位会退化成普通负样本（实测可用率只 ~9%，旧窗口尺寸下）。
        self.pos_oob = []
        if self.oob_neg_ratio > 0:
            self.pos_oob = [t for t in self.pos if self._oob_ok(t[1])]
        self.rng = random.Random(seed)

    @staticmethod
    def _oob_ok(v):
        wh = image_size(IMG_ROOT / v['img'])           # 只读文件头，不解码像素
        if wh is None:
            return False
        w, h = wh
        return oob_feasible(h, w, 144, bbox_center_px(v['bbox'], 1.0, 1.0),
                            seed=abs(hash(v['img'])) % 65536)

    def __len__(self):
        if self.augment:
            return (len(self.pos) + int(len(self.pos) * self.neg_ratio)
                    + int(len(self.pos_oob) * self.oob_neg_ratio))
        return len(self.pos) + len(self.neg) + len(self.neg_extra)

    def _load_pos(self, uuid, v):
        bu, bv = bbox_center_px(v['bbox'], 1.0, 1.0)   # 归一化标注中心
        img, geom = load_jpeg_geom(IMG_ROOT / v['img'], self.size, self.augment, self.rng,
                                   center=(bu, bv), keep_center=self.augment, zoom_min=self.zoom_min)
        u, vv = map_point(bu, bv, geom)
        return img, 1.0, u, vv

    def _load_neg(self, uuid, v):
        img = load_jpeg_geom(IMG_ROOT / v['img'], self.size, self.augment, self.rng)[0]
        return img, 0.0, 0.0, 0.0

    def _load_neg_extra(self, path):
        """域内负样本：用**部署口径**窗口（短边缩放+中心裁剪），只加光度抖动增加多样性。

        刻意不用随机裁剪：部署时相机看到的就是这个中心窗口，
        要压制的正是"中心窗口里没有目标手、conf 却很高"这种误触发。
        """
        img = load_jpeg_geom(path, self.size, False, self.rng)[0]
        if self.augment and self.rng.random() < 0.5:   # 验证/评测走纯部署口径, 不加抖动
            img = photometric(img, self.rng)
        return img, 0.0, 0.0, 0.0

    def _load_oob(self, uuid, v):
        """「目标手不在窗口里」负样本：conf=0、不参与 uv 回归。

        2026-10-15 实测动因：4.8% 的 val 正样本标注中心落在部署中心窗口之外（中位误差 106px、
        100% 超 8px），而训练里 keep_center=True 保证窗口一定包含中心 → 模型从没见过
        "窗口里没有目标手"的输入，也没有"这里没有手"这个输出状态，只能给出一个内部合理点。
        这里用与部署同尺寸的窗口把标注中心推出画面 d in [5%,30%]（见 transforms.geom_oob）。
        推不出去的样本（近方图且手居中）回退成域内负样本，避免制造错误标签。
        """
        bu, bv = bbox_center_px(v['bbox'], 1.0, 1.0)
        img, _ = load_jpeg_oob(IMG_ROOT / v['img'], self.size, self.augment, self.rng, (bu, bv))
        if img is None:
            self.oob_stats['fallback'] += 1
            if self.neg_extra:
                return self._load_neg_extra(self.neg_extra[self.rng.randrange(len(self.neg_extra))])
            i = self.rng.randrange(len(self.neg))
            return self._load_neg(*self.neg[i])
        self.oob_stats['feasible'] += 1
        return img, 0.0, 0.0, 0.0

    def __getitem__(self, idx):
        if idx < len(self.pos):
            uuid, v = self.pos[idx]
            img, conf, u, vv = self._load_pos(uuid, v)
        else:
            k = idx - len(self.pos)
            if self.augment:
                n_neg_slots = int(len(self.pos) * self.neg_ratio)
                if k < n_neg_slots:                 # (1) 普通负样本池（HaGRID no_gesture + 自录域内）
                    kind, i = self.neg_pool[k % max(1, len(self.neg_pool))]
                    img, conf, u, vv = (self._load_neg(*self.neg[i]) if kind == 'hagrid'
                                        else self._load_neg_extra(self.neg_extra[i]))
                else:                               # (2) 目标手在窗口外（只从可用正样本里抽）
                    if self.pos_oob:
                        j = self.rng.randrange(len(self.pos_oob))
                        img, conf, u, vv = self._load_oob(*self.pos_oob[j])
                    else:                           # 一张都造不出 → 退化成普通负样本
                        kind, i = self.neg_pool[k % max(1, len(self.neg_pool))]
                        img, conf, u, vv = (self._load_neg(*self.neg[i]) if kind == 'hagrid'
                                            else self._load_neg_extra(self.neg_extra[i]))
            else:
                kind, i = ('hagrid', k) if k < len(self.neg) else ('rec', k - len(self.neg))
                img, conf, u, vv = (self._load_neg(*self.neg[i]) if kind == 'hagrid'
                                    else self._load_neg_extra(self.neg_extra[i]))
        x = normalize(img).transpose(2, 0, 1).astype(np.float32)
        y = np.array([conf, u, vv], dtype=np.float32)
        return x, y
