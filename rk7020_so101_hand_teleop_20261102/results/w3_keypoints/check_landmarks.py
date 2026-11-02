"""下一轮(关键点)开工前自查: HaGRID 21 点 landmarks 标签到底对不对?

用途: 遵守"数据集必须先打开给主人看"的规矩 —— 生成含 21 点标注的样张拼图。
同时也打印结构性事实(字段、坐标范围、手数)，用于确认坐标是归一化 [0,1] 还是像素。
"""
import io
import json
import zipfile
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
ZIP = ROOT / 'model' / 'data' / 'annotations.zip'
SUB = ROOT / 'model' / 'data' / 'hagrid_subset'
OUT = Path(__file__).resolve().parent
GEST = ['fist', 'palm', 'like', 'ok', 'peace', 'no_gesture']
PER_CLASS = 4
TILE_H = 260

# MediaPipe 手部 21 点骨架连接
EDGES = [(0, 1), (1, 2), (2, 3), (3, 4), (0, 5), (5, 6), (6, 7), (7, 8), (5, 9), (9, 10), (10, 11), (11, 12),
         (9, 13), (13, 14), (14, 15), (15, 16), (13, 17), (17, 18), (18, 19), (19, 20), (0, 17)]


def load_ann(zf, split, gesture):
    raw = zf.read(f'annotations/{split}/{gesture}.json')
    obj = json.loads(raw)
    if isinstance(obj, list):                      # 兼容 list 形式
        obj = {e.get('uuid', str(i)): e for i, e in enumerate(obj)}
    return obj


def norm_pts(hd):
    """兼容 list[list] / list[dict] / 空 的手部点集 -> [(x,y), ...]"""
    out = []
    for pt in (hd or []):
        if isinstance(pt, dict):
            out.append((pt.get('x'), pt.get('y')))
        elif isinstance(pt, (list, tuple)) and len(pt) >= 2:
            out.append((pt[0], pt[1]))
    return [(float(a), float(b)) for a, b in out if a is not None and b is not None]


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    stat = {}
    tiles, labels = [], []
    with zipfile.ZipFile(ZIP) as zf:
        names = [n for n in zf.namelist() if n.endswith('.json')]
        print(f'annotations.zip: {len(names)} 个 json (顶层: {sorted({n.split("/")[1] for n in names if "/" in n})[:6]})')
        for g in GEST:
            try:
                ann = load_ann(zf, 'train', g)
            except KeyError:
                print(f'  [跳过] {g}: zip 内无该 json')
                continue
            imgs = sorted((SUB / g).glob('*.jpg'))
            picked, xs, ys, nhand, nbbox = [], [], [], Counter(), Counter()
            for p in imgs:
                e = ann.get(p.stem)
                if not e:
                    continue
                hl = [h for h in (e.get('hand_landmarks') or []) if len(norm_pts(h)) == 21]
                xs += [p[0] for h in hl for p in norm_pts(h)]
                ys += [p[1] for h in hl for p in norm_pts(h)]
                nhand[len(hl)] += 1
                nbbox[len(e.get('bboxes') or [])] += 1
                if len(picked) < PER_CLASS:
                    picked.append((p, e, hl))
                if len(picked) >= PER_CLASS and len(xs) > 300:
                    break
            stat[g] = {'n_ann': len(ann), 'n_local_img': len(imgs),
                       'x_range': (round(min(xs), 3), round(max(xs), 3)) if xs else None,
                       'y_range': (round(min(ys), 3), round(max(ys), 3)) if ys else None,
                       'n_hands_dist': dict(nhand), 'n_bbox_dist': dict(nbbox)}
            print(f'\n[{g}] 标注 {len(ann)} 条 / 本地图 {len(imgs)} 张 | x∈{stat[g]["x_range"]} y∈{stat[g]["y_range"]}'
                  f' | 手数分布 {dict(nhand)} | bbox数分布 {dict(nbbox)}')
            for p, e, hl in picked:
                bgr = cv2.imread(str(p))
                if bgr is None:
                    continue
                h, w = bgr.shape[:2]
                k = TILE_H / h
                t = cv2.resize(bgr, (max(1, int(round(w * k))), TILE_H))
                for hd in hl:                                   # 多只手都画
                    npts = norm_pts(hd)
                    if len(npts) != 21:
                        continue
                    pts = np.array([[int(x * w * k), int(y * h * k)] for x, y in npts])
                    for a, b in EDGES:
                        cv2.line(t, tuple(pts[a]), tuple(pts[b]), (0, 255, 0), 2, cv2.LINE_AA)
                    for i, pt in enumerate(pts):
                        cv2.circle(t, tuple(pt), 3, (0, 0, 255), -1, cv2.LINE_AA)
                        if i == 0:
                            cv2.circle(t, tuple(pt), 6, (255, 0, 0), 2, cv2.LINE_AA)   # 0 号=手腕
                    bb = (e.get('bboxes') or [None])[0]
                    if bb:
                        cv2.rectangle(t, (int(bb[0] * w * k), int(bb[1] * h * k)),
                                      (int((bb[0] + bb[2]) * w * k), int((bb[1] + bb[3]) * h * k)), (255, 200, 0), 2)
                cv2.putText(t, f'{g} {w}x{h}', (6, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                tiles.append(t)
                labels.append(g)
    if not tiles:
        print('!! 没生成任何样张')
        return
    # 拼成网格: 每行一张手势的 PER_CLASS 张
    per_row = max(1, len(tiles) // len(set(labels)))
    rows = [tiles[i:i + per_row] for i in range(0, len(tiles), per_row)]
    Wmax = max(sum(t.shape[1] for t in r) for r in rows)
    canvas = np.full((len(rows) * TILE_H, Wmax, 3), 30, np.uint8)
    for ri, r in enumerate(rows):
        xx = 0
        for t in r:
            canvas[ri * TILE_H:(ri + 1) * TILE_H, xx:xx + t.shape[1]] = t
            xx += t.shape[1]
    out = OUT / 'samples_landmarks.png'
    cv2.imwrite(str(out), canvas)
    print(f'\n>>> 样张已写出: {out.relative_to(ROOT)}  ({canvas.shape[1]}x{canvas.shape[0]})')
    print(f'    红点=21 个 landmark, 蓝圈=0号(手腕), 绿线=骨架, 黄框=bbox; 行顺序={list(dict.fromkeys(labels))}')
    (OUT / 'landmark_check.json').write_text(json.dumps(stat, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'    结构统计 -> results/w3_keypoints/landmark_check.json')


if __name__ == '__main__':
    main()
