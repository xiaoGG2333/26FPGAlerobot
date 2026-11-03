# -*- coding: utf-8 -*-
"""自采视频 -> MediaPipe 蒸馏打标 -> ROI 裁切缓存（hand-pi 环境）。

与 build_roi_cache.py（HaGRID 图像版）的差异：
- 输入是**一个连续视频**：按 --stride 抽帧，掐掉结尾 --skip-tail 秒
- **IMAGE 模式逐帧独立推理**（不用 VIDEO 跟踪模式，避免跟踪漂移把错误标签带进数据集）
- 划分按**时间连续块**（前 75% train / 中 12.5% val / 后 12.5% test）：
  相邻抽帧只差 133ms，随机划分会让验证指标虚高，必须按块切
- 额外存 frame.npy / t.npy（原帧号、秒），为后续时序一致性损失与视频回放留接口
- ROI：教师 21 点外接方窗 x margin U(1.4, 1.9)，原生分辨率裁切后 INTER_AREA 到 256²
- 可选 --neg-seg/--neg-count：从“确定没有手”的片子里抽负样本（presence=0），
  坐标标签填 0，train_kpt 里已被 `m = pr > 0.5` 屏蔽，只参与 presence BCE

用法（hand-pi）：
  D:/Anaconda3/envs/hand-pi/python.exe -u tools/self_video_cache.py \
      --video model/data/self/raw/WIN_20261102_21_17_08_Pro.mp4 \
      --out results/stage1/cache_self_v1 --stride 4 --skip-tail 5

  第 2 支录像（手由近及远 + 5s 空手负样本，全部进训练）：
  ... --seg 0,60 --stride 3 --neg-seg 60,65 --neg-count 200 --skip-tail 4 --train-all
"""
import argparse
import json
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sw'))
import pose_map  # noqa: E402

DET_MAX = 1280          # 检测分辨率上限（landmark 是归一化坐标，缩放不影响标签）


def parse():
    ap = argparse.ArgumentParser()
    ap.add_argument('--video', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--stride', type=int, default=4)
    ap.add_argument('--skip-tail', type=float, default=5.0)
    ap.add_argument('--size', type=int, default=256)
    ap.add_argument('--min-score', type=float, default=0.7)
    ap.add_argument('--margin-lo', type=float, default=1.4)
    ap.add_argument('--margin-hi', type=float, default=1.9)
    ap.add_argument('--split', default='0.75,0.875')
    ap.add_argument('--seed', type=int, default=31)
    ap.add_argument('--max-frames', type=int, default=0)
    ap.add_argument('--seg', default='', help='正样本时间段 秒 "0,60"（空=整段）')
    ap.add_argument('--neg-seg', default='', help='负样本时间段 秒 "60,65"')
    ap.add_argument('--neg-count', type=int, default=0, help='负样本张数')
    ap.add_argument('--train-all', action='store_true',
                    help='不留 test；train=全部正样本，val 仅取前 64 张做监控信号')
    return ap.parse_args()


def crop_roi(img, side, cx, cy, size):
    """方窗裁切（越界 REPLICATE），返回 crop 与 (x0,y0,side) 像素参数。"""
    H, W = img.shape[:2]
    side = max(side, 8.0)
    lx, ly = cx - side / 2, cy - side / 2
    pl, pr = max(0, -int(np.floor(lx))), max(0, int(np.ceil(lx + side)) - W)
    pt, pb = max(0, -int(np.floor(ly))), max(0, int(np.ceil(ly + side)) - H)
    if pl or pr or pt or pb:
        img = cv2.copyMakeBorder(img, pt, pb, pl, pr, cv2.BORDER_REPLICATE)
        lx, ly = lx + pl, ly + pt
    x0, y0 = int(round(lx)), int(round(ly))
    sd = int(round(side))
    c = img[y0:y0 + sd, x0:x0 + sd]
    if c.size == 0:
        return None, None
    c = cv2.resize(c, (size, size), interpolation=cv2.INTER_AREA)
    return c, (x0 - pl, y0 - pt, sd)      # 去掉 padding 偏移，回到原图坐标


def main():
    a = parse()
    rng = random.Random(a.seed)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(a.video)
    if not cap.isOpened():
        print('[错误] 打不开视频', flush=True)
        return 2
    fps = cap.get(cv2.CAP_PROP_FPS)
    n_all = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    dur = n_all / max(fps, 1e-6)
    n_keep = max(1, int(round((dur - a.skip_tail) * fps)))
    f_lo, f_hi = 0, n_keep
    if a.seg:
        s0, s1 = [float(x) for x in a.seg.split(',')]
        f_lo, f_hi = int(s0 * fps), min(n_keep, int(s1 * fps))
    n_expect = max(0, f_hi - f_lo) // a.stride
    print(f'[视频] {W}x{H} {fps:.3f}fps 总 {n_all} 帧 {dur:.1f}s -> '
          f'保留 {n_keep} 帧（掐尾 {a.skip_tail}s），每 {a.stride} 帧抽 1 -> 约 {n_expect} 张',
          flush=True)

    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions
    lm = vision.HandLandmarker.create_from_options(vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(pose_map.DEFAULT_MODEL)),
        running_mode=vision.RunningMode.IMAGE, num_hands=1))

    imgs, pts, hand, scale, roi, fidx = [], [], [], [], [], []
    cnt = {'seen': 0, 'no_hand': 0, 'low_score': 0, 'bad_roi': 0, 'keep': 0}
    t0 = time.time()
    i = -1
    while True:
        ok, frame = cap.read()
        if not ok or i + 1 >= n_keep:
            break
        i += 1
        if i < f_lo or i >= f_hi:
            continue
        if (i - f_lo) % a.stride:
            continue
        cnt['seen'] += 1
        H_, W_ = frame.shape[:2]
        det = frame
        sc = 1.0
        if max(H_, W_) > DET_MAX:
            sc = DET_MAX / max(H_, W_)
            det = cv2.resize(frame, (int(W_ * sc), int(H_ * sc)))
        res = lm.detect(mp.Image(image_format=mp.ImageFormat.SRGB,
                                 data=np.ascontiguousarray(cv2.cvtColor(det, cv2.COLOR_BGR2RGB))))
        if not res.hand_landmarks:
            cnt['no_hand'] += 1
            continue
        if float(res.handedness[0][0].score) < a.min_score:
            cnt['low_score'] += 1
            continue
        p = np.array([[q.x, q.y] for q in res.hand_landmarks[0]], np.float32)   # 归一化
        px = p * np.array([W_, H_], np.float32)                                 # 全帧像素
        xy0, xy1 = px.min(axis=0), px.max(axis=0)
        side = float(np.max(xy1 - xy0)) * rng.uniform(a.margin_lo, a.margin_hi)
        cxy = (xy0 + xy1) / 2 + np.array([rng.uniform(-0.04, 0.04) * side,
                                          rng.uniform(-0.04, 0.04) * side], np.float32)
        crop, box = crop_roi(frame, side, float(cxy[0]), float(cxy[1]), a.size)
        if crop is None:
            cnt['bad_roi'] += 1
            continue
        x0, y0, sd = box
        pr = (px - np.array([x0, y0], np.float32)) / sd
        if not (np.all(pr > -0.35) and np.all(pr < 1.35)):
            cnt['bad_roi'] += 1
            continue
        cnt['keep'] += 1
        imgs.append(crop)
        pts.append(pr.astype(np.float32))
        hand.append({'Left': 1, 'Right': 2}.get(str(res.handedness[0][0].category_name), 0))
        scale.append(np.linalg.norm(pr[9] - pr[0]))
        roi.append((x0 / W_, y0 / H_, sd / W_, sd / H_))
        fidx.append(i)
        if cnt['seen'] % 200 == 0:
            print(f'  [{cnt["seen"]}/{n_expect}] keep={cnt["keep"]} '
                  f'{cnt["seen"]/(time.time()-t0):.1f} fps', flush=True)
        if a.max_frames and cnt['keep'] >= a.max_frames:
            break
    cap.release()
    n_pos = len(imgs)                         # 正样本张数（负样本追加在其后）
    sides_pos = [r[2] * W for r in roi]        # 正样本方窗边长（像素），给负样本对齐尺度
    got = 0
    # ---------- 负样本：从确认无手的片段抽 ----------
    if a.neg_seg and a.neg_count > 0:
        s0, s1 = [float(x) for x in a.neg_seg.split(',')]
        g0, g1 = int(s0 * fps), min(n_keep, int(s1 * fps))
        med_side = float(np.median(sides_pos)) if sides_pos else 500.0
        plan = []
        for _ in range(a.neg_count):
            fid = rng.randrange(max(g0, 0), max(g1, g0 + 1))
            side = med_side * rng.uniform(0.9, 1.35)
            cx = rng.uniform(side / 2, max(side / 2 + 1, W - side / 2))
            cy = rng.uniform(side / 2, max(side / 2 + 1, H - side / 2))
            plan.append((fid, side, cx, cy))
        plan.sort()
        cap2 = cv2.VideoCapture(a.video)
        cap2.set(cv2.CAP_PROP_POS_FRAMES, g0)      # 必须 seek，否则 cur 计数与实际读到的帧错位
        cur = g0 - 1
        ok = True
        det_cache = {}
        n_rej = 0
        for fid, side, cx, cy in plan:
            while cur < fid:
                ok, fr = cap2.read()
                cur += 1
                if not ok:
                    break
            if not ok:
                break
            if fid not in det_cache:      # 负样本必须真没手：逐帧用 MediaPipe 兜底复核
                dd = fr
                if max(fr.shape[:2]) > DET_MAX:
                    kk = DET_MAX / max(fr.shape[:2])
                    dd = cv2.resize(fr, (int(fr.shape[1] * kk), int(fr.shape[0] * kk)))
                rr = lm.detect(mp.Image(image_format=mp.ImageFormat.SRGB,
                                        data=np.ascontiguousarray(cv2.cvtColor(dd, cv2.COLOR_BGR2RGB))))
                det_cache[fid] = (len(rr.hand_landmarks) == 0)
            if not det_cache[fid]:
                n_rej += 1
                continue
            crop, box = crop_roi(fr, side, cx, cy, a.size)
            if crop is None:
                continue
            x0, y0, sd = box
            imgs.append(crop)
            pts.append(np.full((21, 2), 0.5, np.float32))
            hand.append(0)
            scale.append(0.0)
            roi.append((x0 / W, y0 / H, sd / W, sd / H))
            fidx.append(int(fid))
            got += 1
            if got % 50 == 0:
                print(f'  [负样本 {got}/{a.neg_count}]', flush=True)
        cap2.release()
        print(f'[负样本] {got} 张（帧 {g0}..{g1}，方窗中位数 {med_side:.0f}px，'
              f'MediaPipe 复核拒掉 {n_rej}）', flush=True)
    pose_map.close_quietly(lm)
    print(f'[统计] {cnt}  用时 {time.time()-t0:.0f}s', flush=True)

    N = len(imgs)
    arr = np.stack(imgs)
    scale = np.array(scale, np.float32)
    P = np.stack(pts)
    fidx = np.array(fidx, np.int64)
    pres = np.zeros(N, np.float32)
    pres[:n_pos] = 1.0
    # 划分：默认按时间连续块（不是随机划分）；--train-all 时全进训练，留 64 张做监控
    if a.train_all:
        v = min(64, n_pos)
        split_idx = {'train': list(range(0, n_pos)), 'val': list(range(0, v)),
                     'test': [], 'neg': list(range(n_pos, N))}
    else:
        f1, f2 = [float(x) for x in a.split.split(',')]
        b1, b2 = int(n_pos * f1), int(n_pos * f2)
        split_idx = {'train': list(range(0, b1)), 'val': list(range(b1, b2)),
                     'test': list(range(b2, n_pos)), 'neg': list(range(n_pos, N))}
    np.save(out / 'imgs.npy', arr)
    np.save(out / 'pts.npy', P)
    np.save(out / 'presence.npy', pres)
    np.save(out / 'hand.npy', np.array(hand, np.int64))
    np.save(out / 'scale.npy', scale)
    np.save(out / 'roi.npy', np.array(roi, np.float32))
    np.save(out / 'frame.npy', fidx)
    np.save(out / 't.npy', (fidx / fps).astype(np.float32))
    (out / 'split.json').write_text(json.dumps(split_idx), encoding='utf-8')

    meta = {'source': str(a.video), 'size': a.size, 'n_samples': N,
            'video': {'w': W, 'h': H, 'fps': round(fps, 3), 'frames': n_all,
                      'dur_s': round(dur, 1), 'skip_tail_s': a.skip_tail,
                      'stride': a.stride, 'frames_kept': n_keep},
            'counts': {k: len(v) for k, v in split_idx.items()},
            'filter_stats': cnt, 'split_mode': 'train_all' if a.train_all else 'contiguous_time_block',
            'margin': [a.margin_lo, a.margin_hi], 'min_score': a.min_score,
            'seg': a.seg, 'neg_seg': a.neg_seg, 'n_neg': len(split_idx['neg']),
            'hand_pct': [round(float(x), 4) for x in np.percentile(scale, [5, 25, 50, 75, 95])],
            'created': time.strftime('%Y-%m-%d %H:%M:%S')}
    (out / 'meta.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')
    print(f"[手长/ROI] 分位 {meta['hand_pct']}")
    print('[统计] ' + json.dumps(meta['counts']))

    # 自检贴图（12 张，含教师点，供我们自己核对管线没歪）
    sel = np.linspace(0, N - 1, 12).astype(int)
    tiles = []
    for k in sel:
        t = arr[k].copy()
        if pres[k] > 0.5:
            q = P[k] * a.size
            for u, v in pose_map.HAND_LINKS:
                cv2.line(t, tuple(np.int32(q[u])), tuple(np.int32(q[v])), (0, 255, 0), 1, cv2.LINE_AA)
            for j in range(21):
                cv2.circle(t, tuple(np.int32(q[j])), 2, (0, 0, 255), -1)
        else:
            cv2.putText(t, 'NEG', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        tiles.append(t)
    rows = [np.hstack(tiles[i:i + 6]) for i in range(0, 12, 6)]
    cv2.imwrite(str(out / 'self_check.png'), np.vstack(rows))
    print(f'[自检图] {out / "self_check.png"}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
