# -*- coding: utf-8 -*-
"""录制部署机位视频（train-pi / hand-pi 均可，纯 cv2）。

用途：在**最终部署相机位置**上采数据，交给 tools/self_video_cache.py 打标。

用法：
  python -u tools/cap_cam.py --sec 120 --out model/data/self/raw/cam640_01.avi
  python -u tools/cap_cam.py --sec 120 --roi 320,240,288      # 叠加固定框辅助摆位
热键：q / ESC 提前结束，空格 暂停/继续。
"""
import argparse
import time
from pathlib import Path

import cv2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cam', type=int, default=0)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--sec', type=float, default=60.0)
    ap.add_argument('--out', required=True)
    ap.add_argument('--roi', default='', help='画一个参考框 cx,cy,side（不参与录制内容）')
    ap.add_argument('--no-preview', action='store_true')
    args = ap.parse_args()

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    cap = cv2.VideoCapture(args.cam, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise SystemExit(f'打不开相机 {args.cam}')
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or args.fps
    wr = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*'MJPG'), fps, (W, H))
    print(f'[采集] {W}x{H} @ {fps:.1f}fps -> {out}  计划 {args.sec:.0f}s')

    preview = not args.no_preview
    roi = [float(x) for x in args.roi.split(',')] if args.roi else None
    t0 = time.time()
    n, paused = 0, False
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if not paused:
            wr.write(f)
            n += 1
            if n % 30 == 0:
                print(f'  {n} 帧  {time.time()-t0:5.1f}s', flush=True)
        if preview:
            v = f.copy()
            if roi:
                x0, y0 = int(roi[0] - roi[2] / 2), int(roi[1] - roi[2] / 2)
                cv2.rectangle(v, (x0, y0), (x0 + int(roi[2]), y0 + int(roi[2])), (60, 200, 255), 2)
            el = time.time() - t0
            cv2.putText(v, f'{el:5.1f}/{args.sec:.0f}s  {n} frames'
                           + ('  PAUSED' if paused else ''), (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            try:
                cv2.imshow('cap_cam', v)
            except cv2.error as e:
                print('[提醒] 这个 cv2 没有 GUI 支持（headless 版），关掉预览只录制。\n'
                      '        想边看边录请用 /d/Anaconda3/envs/train-pi/python.exe 或 hand-pi 的 python。',
                      flush=True)
                preview = False
                continue
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('q')):
                break
            if k == 32:
                paused = not paused
        if time.time() - t0 >= args.sec:
            break
    cap.release()
    wr.release()
    if preview:
        cv2.destroyAllWindows()
    print(f'[完成] {n} 帧 -> {out}')


if __name__ == '__main__':
    main()
