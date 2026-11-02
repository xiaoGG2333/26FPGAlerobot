# -*- coding: utf-8 -*-
"""阶段1 M0：用 MediaPipe 当教师，给视频逐帧自动打标（IMAGE 模式，独立推理）。

为什么 IMAGE 模式：VIDEO 模式带跟踪，一旦漂移会把错误标签成片带进数据集（计划书 §4.3）。
手长口径与 pose_map.Geometry 完全一致：hand_px = ‖p9-p0‖（归一化坐标，MIN_HAND_PX=0.08）。

用法（hand-pi 环境，禁装进其它环境）：
  D:/Anaconda3/envs/hand-pi/python.exe -u sw/label_auto.py                     # 全量 calib_rec.mp4
  D:/Anaconda3/envs/hand-pi/python.exe -u sw/label_auto.py --max-frames 120    # 冒烟
输出：
  results/stage1/calib_rec_labels.npz        pts(N,21,2) / score / hand_code / valid / discard
  results/stage1/calib_rec_labels_meta.json  视频与过滤参数、统计
  results/stage1/calib_rec_discard.json      丢弃清单（帧号按原因分组，反查教师缺陷用）
  results/stage1/label_check_*.png           抽样 contact sheet（硬规矩：必须贴图给主人看）
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sw'))
import pose_map  # 只取常量与工具（mediapipe 在其 main() 里才 import，同进程安全）

HAND_LINKS = pose_map.HAND_LINKS

# discard 原因码
R_KEEP, R_NOHAND, R_LOWSCORE, R_SMALL, R_JUMP = 0, 1, 2, 3, 4
REASON_NAME = {R_KEEP: 'keep', R_NOHAND: 'no_hand', R_LOWSCORE: 'low_score',
               R_SMALL: 'hand_small', R_JUMP: 'jump'}


def parse_args():
    ap = argparse.ArgumentParser(description='MediaPipe 教师自动打标')
    ap.add_argument('--video', default=str(ROOT / 'results/w3_pose/calib_rec.mp4'))
    ap.add_argument('--model', default=str(ROOT / 'model/data/external/mediapipe/hand_landmarker.task'))
    ap.add_argument('--out-dir', default=str(ROOT / 'results/stage1'))
    ap.add_argument('--min-score', type=float, default=0.8)
    ap.add_argument('--min-hand', type=float, default=pose_map.MIN_HAND_PX)   # 0.08，与部署同口径
    ap.add_argument('--max-jump', type=float, default=0.15)
    ap.add_argument('--max-frames', type=int, default=0, help='0=整段')
    ap.add_argument('--skip-frames', type=int, default=0, help='跳过前 N 帧（分段探查用）')
    ap.add_argument('--sheet', type=int, default=100, help='contact sheet 抽样帧数（0=不出图）')
    return ap.parse_args()


def detect_pass(video, model_path, args, log):
    """第一遍：逐帧推理 + score/手长过滤；返回逐帧记录列表（不存图像，省内存）。"""
    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions

    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        log(f'[错误] 打不开视频: {video}')
        return None, None, None
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(cap.get(cv2.CAP_PROP_FPS) or 30.0)
    log(f'[视频] {video.name}  {W}x{H} @{fps:.1f}fps')

    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.IMAGE,
        num_hands=1,
        min_hand_detection_confidence=0.5)
    landmarker = vision.HandLandmarker.create_from_options(opts)
    log(f'[模型] HandLandmarker IMAGE 模式已加载 ({model_path.stat().st_size/1e6:.2f} MB)')

    recs = []          # 每帧一条 dict
    t0 = time.time()
    idx = 0
    skipped = 0
    while skipped < args.skip_frames:
        ok, _ = cap.read()
        if not ok:
            break
        skipped += 1
    if skipped:
        log(f'[跳过] 前 {skipped} 帧')
    idx = skipped          # 帧号保持视频内绝对序号（丢弃清单/抽查图要对得上）
    while True:
        if args.max_frames and len(recs) >= args.max_frames:
            break
        ok, frame = cap.read()
        if not ok:
            break
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        res = landmarker.detect(mp_img)

        rec = {'idx': idx, 'found': False, 'pts': None, 'score': 0.0,
               'hand': '', 'hp': 0.0, 'reason': R_NOHAND}
        if res.hand_landmarks:
            lm = res.hand_landmarks[0]
            pts = np.array([[p.x, p.y] for p in lm], dtype=np.float32)   # (21,2) 归一化
            score = float(res.handedness[0][0].score)
            name = str(res.handedness[0][0].category_name)
            hp = float(np.linalg.norm(pts[9] - pts[0]))                  # 与 pose_map 同口径
            rec.update(found=True, pts=pts, score=score, hand=name, hp=hp)
            if score < args.min_score:
                rec['reason'] = R_LOWSCORE
            elif hp < args.min_hand:
                rec['reason'] = R_SMALL
            else:
                rec['reason'] = R_KEEP
        recs.append(rec)
        idx += 1
        if idx % 500 == 0:
            el = time.time() - t0
            kept = sum(1 for r in recs if r['reason'] == R_KEEP)
            log(f'[进度] {idx} 帧  {idx/el:.1f} fps  暂存 {kept}')

    el = time.time() - t0
    cap.release()
    log(f'[推理] 共 {idx} 帧，耗时 {el:.1f}s（{idx/max(el,1e-6):.1f} fps）')
    return recs, {'W': W, 'H': H, 'fps': fps, 'frames': idx, 'infer_seconds': round(el, 1)}, landmarker


def jump_filter(recs, max_jump):
    """第二遍：与前后"通过过滤的相邻帧"比最大点位移；两侧都超阈值才判跳变（快速运动不误杀）。"""
    ok_idx = [i for i, r in enumerate(recs) if r['reason'] == R_KEEP]
    n_jump = 0
    for k, i in enumerate(ok_idx):
        d_prev = d_next = None
        if k > 0:
            j = ok_idx[k - 1]
            d_prev = float(np.abs(recs[i]['pts'] - recs[j]['pts']).max())
        if k + 1 < len(ok_idx):
            j = ok_idx[k + 1]
            d_next = float(np.abs(recs[i]['pts'] - recs[j]['pts']).max())
        cand = [d for d in (d_prev, d_next) if d is not None]
        if cand and min(cand) > max_jump:
            recs[i]['reason'] = R_JUMP
            n_jump += 1
    return n_jump


def build_arrays(recs, n):
    pts = np.zeros((n, 21, 2), dtype=np.float32)
    score = np.zeros(n, dtype=np.float32)
    hand_code = np.zeros(n, dtype=np.uint8)          # 0无 / 1=Left / 2=Right（教师原始输出，swap 未验证）
    valid = np.zeros(n, dtype=bool)
    discard = np.full(n, R_NOHAND, dtype=np.uint8)
    hand_len = np.zeros(n, dtype=np.float32)
    for i, r in enumerate(recs):
        score[i] = r['score']
        hand_len[i] = r['hp']
        hand_code[i] = {'Left': 1, 'Right': 2}.get(r['hand'], 0)
        if r['reason'] == R_KEEP:
            pts[i] = r['pts']
            valid[i] = True
            discard[i] = R_KEEP
        else:
            discard[i] = r['reason']
            if r['found']:
                pts[i] = r['pts']        # 被过滤的检测结果也存（复查教师用），valid=False
    return pts, score, hand_code, valid, discard, hand_len


def draw_tile(frame, rec, tw=320):
    """缩略图 + 骨架叠加：keep=绿，被丢=红（标原因）。"""
    H, W = frame.shape[:2]
    th = max(1, int(round(tw * H / W)))
    img = cv2.resize(frame, (tw, th))
    if rec is not None and rec['found'] and rec['pts'] is not None:
        col = (80, 220, 80) if rec['reason'] == R_KEEP else (60, 60, 230)
        px = (rec['pts'] * np.array([tw, th], dtype=np.float32)).astype(int)
        for a, b in HAND_LINKS:
            cv2.line(img, tuple(px[a]), tuple(px[b]), col, 1, cv2.LINE_AA)
        for j in range(21):
            cv2.circle(img, tuple(px[j]), 2, col, -1, cv2.LINE_AA)
    if rec is not None:
        tag = '#%d' % rec['idx']
        if rec['found']:
            tag += ' s=%.2f hp=%.3f %s' % (rec['score'], rec['hp'],
                                           'OK' if rec['reason'] == R_KEEP else REASON_NAME[rec['reason']])
        else:
            tag += ' no_hand'
        cv2.putText(img, tag, (4, th - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img, tag, (3, th - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 2, cv2.LINE_AA)
    return img


def make_sheet(video, recs, picks, out_png, cols=10, tw=320, title=''):
    """重开视频按 picks（升序帧号）取帧，拼 contact sheet。"""
    want = set(picks)
    cap = cv2.VideoCapture(str(video))
    got = {}
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx in want:
            got[idx] = frame
            if len(got) == len(want):
                break
        idx += 1
    cap.release()

    tiles = []
    for i in picks:
        r = next((r for r in recs if r['idx'] == i), None)
        f = got.get(i)
        if f is None:
            continue
        tiles.append(draw_tile(f, r, tw))
    if not tiles:
        return 0
    th = tiles[0].shape[0]
    rows = (len(tiles) + cols - 1) // cols
    head = 26 if title else 0
    canvas = np.full((rows * th + head, cols * tw, 3), 30, np.uint8)
    if title:
        cv2.putText(canvas, title, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    for k, t in enumerate(tiles):
        y = head + (k // cols) * th
        x = (k % cols) * tw
        canvas[y:y + th, x:x + tw] = t
    cv2.imwrite(str(out_png), canvas)
    return len(tiles)


def main():
    args = parse_args()
    log = lambda m: print(m, flush=True)
    video = Path(args.video)
    model_path = Path(args.model)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if not video.exists():
        log(f'[错误] 视频不存在: {video}')
        return 2
    if not model_path.exists():
        log(f'[错误] 模型不存在: {model_path}')
        return 2

    recs, vinfo, landmarker = detect_pass(video, model_path, args, log)
    if recs is None:
        return 2
    n = len(recs)
    n_jump = jump_filter(recs, args.max_jump)
    log(f'[跳变过滤] 剔除 {n_jump} 帧（max_jump={args.max_jump}）')

    pts, score, hand_code, valid, discard, hand_len = build_arrays(recs, n)
    stem = video.stem
    npz_path = out_dir / f'{stem}_labels.npz'
    np.savez_compressed(npz_path, pts=pts, score=score, hand_code=hand_code,
                        valid=valid, discard=discard, hand_len=hand_len)
    log(f'[保存] {npz_path}  ({npz_path.stat().st_size/1e6:.1f} MB)')

    counts = {REASON_NAME[c]: int((discard == c).sum()) for c in sorted(REASON_NAME)}
    keep_rate = counts['keep'] / max(n, 1)
    discard_by_reason = {REASON_NAME[c]: [int(r['idx']) for r in recs if r['reason'] == c]
                         for c in (R_NOHAND, R_LOWSCORE, R_SMALL, R_JUMP)}
    (out_dir / f'{stem}_discard.json').write_text(
        json.dumps(discard_by_reason, ensure_ascii=False), encoding='utf-8')

    import mediapipe as mp
    meta = {'video': str(video), 'video_info': vinfo,
            'filters': {'min_score': args.min_score, 'min_hand': args.min_hand,
                        'max_jump': args.max_jump},
            'counts': counts, 'keep_rate': round(keep_rate, 4),
            'handedness_dist': {k: int(v) for k, v in
                                zip(('none', 'Left', 'Right'),
                                    (int((hand_code == 0).sum()), int((hand_code == 1).sum()),
                                     int((hand_code == 2).sum())))},
            'handedness_note': '教师原始输出；镜像自拍视角 swap 未验证（B1 遗留项），下游使用前需核对',
            'mediapipe_version': mp.__version__,
            'created': time.strftime('%Y-%m-%d %H:%M:%S')}
    (out_dir / f'{stem}_labels_meta.json').write_text(
        json.dumps(meta, ensure_ascii=False, indent=1), encoding='utf-8')

    log('[统计] ' + '  '.join(f'{k}={v}' for k, v in counts.items()) +
        f'  保留率={keep_rate*100:.1f}%')
    if keep_rate < 0.70:
        log('[警告] 保留率 <70%：先停下查教师/视频，不要直接拿去训练（交接文档 T2 验收线）')

    if args.sheet > 0:
        k = min(args.sheet, n)
        head = list(range(min(12, n)))
        mid = list(range(max(0, n // 2 - 6), min(n, n // 2 + 6)))
        tail = list(range(max(0, n - 12), n))
        even = [int(i) for i in np.linspace(0, n - 1, k)]
        picks = sorted(set(head + mid + tail + even))[:max(k, 36)]
        stat = ('%s  %dx%d  %d frames  keep=%.1f%%  (%s)' %
                (video.name, vinfo['W'], vinfo['H'], n, keep_rate * 100,
                 ' '.join(f'{a}={b}' for a, b in counts.items() if a != 'keep')))
        m = make_sheet(video, recs, picks, out_dir / f'label_check_{stem}.png',
                       title=stat)
        log(f'[抽查图] {out_dir / f"label_check_{stem}.png"}  ({m} 帧)')
        d_idx = [r['idx'] for r in recs if r['reason'] != R_KEEP and r['found']]
        if d_idx:
            d_picks = sorted(set([d_idx[int(i)] for i in
                                  np.linspace(0, len(d_idx) - 1, min(50, len(d_idx)))]))
            m2 = make_sheet(video, recs, d_picks, out_dir / f'label_check_{stem}_discard.png',
                            title='discarded detections sample (red): low_score / hand_small / jump')
            log(f'[抽查图] {out_dir / f"label_check_{stem}_discard.png"}  ({m2} 帧)')

    log('[完成] 打标结束。请把 label_check_*.png 贴给主人过目后再进入训练（硬规矩）。')
    pose_map.close_quietly(landmarker)   # mediapipe close() 可能卡 ~40s，限时不等
    sys.stdout.flush()
    os._exit(0)


if __name__ == '__main__':
    sys.exit(main())
