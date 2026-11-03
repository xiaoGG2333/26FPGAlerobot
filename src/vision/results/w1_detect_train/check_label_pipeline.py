"""验证：训练裁剪是否还会把标注中心裁出画面（标签管线 bug 回归测试）。

用法（在 model/ 下）:
    D:/Anaconda3/envs/fpga_amd-pi/python.exe ../results/w1_detect_train/check_label_pipeline.py
"""
import json
import random
import sys
from pathlib import Path

import cv2

MODEL = Path(__file__).resolve().parents[2] / 'model'
sys.path.insert(0, str(MODEL))
from common.transforms import geom_params, map_point, bbox_center_px   # noqa: E402

SRC = MODEL / 'data'
N = 1500


def main():
    data = json.loads((SRC / 'hagrid_subset' / 'dataset.json').read_text(encoding='utf-8'))
    pos = [(u, v) for u, v in data['train'].items() if v['label'] in ('fist', 'palm')][:N]
    print(f'样本 {len(pos)}（train 正样本，augment=True 随机裁剪）')

    dims = {}
    for _, v in pos:                                   # 每张图只读一次尺寸
        p = SRC / v['img']
        if str(p) not in dims:
            im = cv2.imread(str(p))
            dims[str(p)] = im.shape[:2]
    shapes = sorted(set(dims.values()))
    print('原图尺寸集合:', shapes)

    for keep in (False, True):
        rng = random.Random(0)
        oob, vmin, vmax = 0, 9.9, -9.9
        for _, v in pos:
            h, w = dims[str(SRC / v['img'])]
            bu, bv = bbox_center_px(v['bbox'], 1.0, 1.0)
            geom = geom_params(h, w, 144, True, rng, center=(bu, bv), keep_center=keep)
            u, vv = map_point(bu, bv, geom)
            if not (0.0 <= u <= 1.0 and 0.0 <= vv <= 1.0):
                oob += 1
            vmin, vmax = min(vmin, vv), max(vmax, vv)
        tag = 'keep_center=True (修复后)' if keep else 'keep_center=False (旧行为)'
        print(f'{tag}: 标签出界 {oob}/{len(pos)} = {100*oob/len(pos):.2f}%  '
              f'v 范围 {vmin:.3f}~{vmax:.3f}')


if __name__ == '__main__':
    main()
