# -*- coding: utf-8 -*-
"""阶段1 M0/M2：引导式域内素材录制（hand-pi 环境）。

为什么需要它：calib_rec.mp4 实测全域外（人手离镜头太远、hp≈0.05~0.075，部署域要求 0.14~0.20），
见 results/stage1_smoke/label_check_calib_rec.png。sanity 与 M2 都需要域内素材。

用法：
  D:/Anaconda3/envs/hand-pi/python.exe -u sw/collect_data.py --source auto
  （先用 sw/pose_map.py --list-cams 看序号；外接摄像头优先）
画面提示：顶部当前动作提示+倒计时；中部手长条（绿带=目标区 0.14~0.20 归一化手长）。
存的是**原始帧**（无骨架叠加），分段时刻表落 <out>.json 供后续分层抽样。
q/ESC 提前结束。录完看打印的 in-band 比例，>=70% 才算合格素材。
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'sw'))
import pose_map  # noqa: E402

SCRIPT = [
    ('WARMUP: put ONE open palm facing camera, adjust distance until bar in GREEN', 12),
    ('MOVE hand LEFT <-> RIGHT (slow, full width)', 20),
    ('MOVE hand UP <-> DOWN (slow, full height)', 20),
    ('MOVE hand NEAR <-> FAR (palm size changes)', 20),
    ('ROTATE wrist in image plane +-90 deg', 20),
    ('FIST <-> OPEN PALM cycles (2s each)', 20),
    ('FAST shake / wave (motion blur coverage)', 10),
    ('LEFT hand only, repeat small motions', 15),
    ('RIGHT hand only, repeat small motions', 15),
    ('REST: hand out of frame', 5),
]
HP_LO, HP_HI = 0.12, 0.22          # 目标带（部署域 90~130px@640 = 0.14~0.20，留余量）


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='auto')
    ap.add_argument('--out', default=str(ROOT / 'results/stage1/rec_sanity.mp4'))
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--model', default=str(ROOT / 'model/data/external/mediapipe/hand_landmarker.task'))
    args = ap.parse_args()

    import mediapipe as mp
    from mediapipe.tasks.python import vision, BaseOptions

    if args.source == 'auto':
        cams = pose_map.enum_cams() if hasattr(pose_map, 'enum_cams') else []
        print('[提示] auto 需要序号：先跑 sw/pose_map.py --list-cams；或直接用 --source 1')
        if not cams:
            args.source = '1'
    cap = pose_map.open_capture(args.source, args.width, args.height, args.fps)
    if cap is None or not cap.isOpened():
        print(f'[错误] 打不开摄像头: {args.source}')
        return 2

    opts = vision.HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=str(args.model)),
        running_mode=vision.RunningMode.IMAGE, num_hands=1)
    landmarker = vision.HandLandmarker.create_from_options(opts)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'mp4v'),
                         args.fps, (args.width, args.height))
    win = 'collect_data - raw recording'
    cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

    timeline, hp_in, hp_seen, n_frames = [], 0, 0, 0
    t_start = time.time()
    seg_i, seg_t0 = 0, time.time()
    ts_ms = 0
    print('[录制] 开始。q/ESC 结束。', flush=True)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame.shape[:2] != (args.height, args.width):
            frame = cv2.resize(frame, (args.width, args.height))
        now = time.time()
        if seg_i < len(SCRIPT) and now - seg_t0 >= SCRIPT[seg_i][1]:
            timeline.append({'prompt': SCRIPT[seg_i][0], 't0': round(seg_t0 - t_start, 2),
                             't1': round(now - t_start, 2)})
            seg_i += 1
            seg_t0 = now
            print(f'[段] {timeline[-1]["prompt"][:40]}...', flush=True)
        if seg_i >= len(SCRIPT):
            break

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ts_ms += int(1000 / args.fps)
        res = landmarker.detect(mp.Image(image_format=mp.ImageFormat.SRGB,
                                         data=np.ascontiguousarray(rgb)))
        hp = 0.0
        if res.hand_landmarks:
            p = np.array([[q.x, q.y] for q in res.hand_landmarks[0]], np.float32)
            hp = float(np.linalg.norm(p[9] - p[0]))
            hp_seen += 1
            if HP_LO <= hp <= HP_HI:
                hp_in += 1
        vw.write(frame)
        n_frames += 1

        view = frame.copy()
        prompt, dur = SCRIPT[min(seg_i, len(SCRIPT) - 1)]
        left = max(0.0, dur - (now - seg_t0))
        cv2.putText(view, f'{seg_i+1}/{len(SCRIPT)} {left:4.1f}s  {prompt}', (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        bw = args.width - 40
        cv2.rectangle(view, (20, args.height - 34), (20 + bw, args.height - 18),
                      (60, 60, 60), -1)
        x0 = 20 + int(bw * HP_LO / 0.30)
        x1 = 20 + int(bw * HP_HI / 0.30)
        cv2.rectangle(view, (x0, args.height - 34), (x1, args.height - 18), (60, 180, 60), -1)
        if hp > 0:
            xm = 20 + int(bw * min(hp, 0.30) / 0.30)
            cv2.line(view, (xm, args.height - 38), (xm, args.height - 14), (255, 255, 255), 2)
        cv2.putText(view, f'hand_len={hp:.3f} (green band {HP_LO}-{HP_HI})  frames={n_frames}',
                    (8, args.height - 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(win, view)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break

    if seg_i < len(SCRIPT):
        timeline.append({'prompt': SCRIPT[seg_i][0] + ' (cut short)',
                         't0': round(seg_t0 - t_start, 2), 't1': round(time.time() - t_start, 2)})
    cap.release()
    vw.release()
    cv2.destroyAllWindows()
    pose_map.close_quietly(landmarker)
    ratio = hp_in / max(hp_seen, 1)
    meta = {'video': str(out), 'fps': args.fps, 'size': [args.width, args.height],
            'frames': n_frames, 'seconds': round(time.time() - t_start, 1),
            'hand_seen_frames': hp_seen, 'in_band_frames': hp_in,
            'in_band_ratio': round(ratio, 3), 'timeline': timeline}
    Path(str(out) + '.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1),
                                        encoding='utf-8')
    print(f'[完成] {out}  {n_frames} 帧  手检出 {hp_seen}  in-band {hp_in} '
          f'({ratio*100:.0f}%, 目标>=70%)', flush=True)
    if ratio < 0.70:
        print('[警告] in-band 比例不足：重录时把手凑近摄像头，让白条落在绿带里')
    sys.stdout.flush()
    os_exit = getattr(__import__('os'), '_exit')
    os_exit(0)


if __name__ == '__main__':
    sys.exit(main())
