"""实时摄像头实测：DetectNet(144) + ClassifyNet(80) 冻结权重 -> 手势判定 + 夹爪动作建议。

用途：接电脑摄像头，实时看昨天那两个模型在真实镜头下的表现（定位/置信度/三类手势/稳定性）。

口径严格对齐部署管线（与训练评测一致）：
  DetectNet 输入 = 短边缩到 144 -> 中心 144x144 裁剪 -> /255 + ImageNet 归一化(RGB)
  DetectNet 输出 = (conf, u, v)，u,v 是**该 144 裁剪窗口内**的归一化坐标
  映射回原帧     = x = (W - min(W,H))/2 + u*min(W,H)，y 同理
                  （即"检测器只看得见画面中心的正方形"，屏幕上有标注）
  ClassifyNet 输入 = 以预测中心为中心、边长 roi_frac*短边 的方框，再经 crop_roi(expand=1.5) -> 80x80
  类别          = 0 闭合(fist) / 1 张开(palm) / 2 其他(保持不动)

按键：q/ESC 退出   s 存快照   d 显示/隐藏小图   [ ] 调 ROI 边长
      t/T 调置信度阈值  k 清空平滑与投票   r 开始/停止录像
命令行：
  --source 0            摄像头序号（或给视频文件路径）
  --no-window           不开窗口（自检用）
  --record 路径.mp4      全程录像（带标注）
  --out 目录            快照/日志输出目录，默认 results/w2_live
"""
import argparse
import json
import sys
import time
from collections import Counter, deque
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]          # FPGA_AMD
sys.path.insert(0, str(ROOT / 'model'))             # 让 common/detect/classify 可导入
from common.transforms import normalize             # noqa: E402
from detect.net import DetectNet                    # noqa: E402
from classify.net import ClassifyNet                # noqa: E402
from classify.dataset import crop_roi               # noqa: E402

DET_SIZE, CLS_SIZE = 144, 80
CLS_NAME = {0: ('闭合', 'fist / 夹爪->闭合'), 1: ('张开', 'palm / 夹爪->张开'), 2: ('其他', 'other / 夹爪保持')}
CLS_ASCII = {0: 'CLOSED', 1: 'OPEN', 2: 'OTHER'}


def make_painter():
    """有 PIL+中文字体就用中文，否则退回 ASCII(cv2.putText 不支持中文)。"""
    try:
        from PIL import Image, ImageDraw, ImageFont
        for fp in ('C:/Windows/Fonts/msyh.ttc', 'C:/Windows/Fonts/simhei.ttf'):
            if Path(fp).exists():
                cache = {}

                def get(size):
                    if size not in cache:
                        cache[size] = ImageFont.truetype(fp, size)
                    return cache[size]

                class P:
                    cjk = True

                    @staticmethod
                    def flush(bgr, texts):
                        img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                        d = ImageDraw.Draw(img)
                        for t, xy, col, size in texts:
                            x, y = xy
                            d.text((x + 1, y + 1), t, font=get(size), fill=(0, 0, 0))   # 描边
                            d.text((x, y), t, font=get(size), fill=(col[2], col[1], col[0]))
                        return cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
                return P
    except Exception:
        pass

    class P:
        cjk = False

        @staticmethod
        def flush(bgr, texts):
            for t, xy, col, size in texts:
                s = size / 30.0
                cv2.putText(bgr, t, xy, cv2.FONT_HERSHEY_SIMPLEX, s, col, 2, cv2.LINE_AA)
            return bgr
    return P


def detect_input(rgb, size=DET_SIZE):
    """部署管线：短边缩到 size -> 中心 size×size 裁剪。返回 (裁剪图, 缩放比 s, 原图边长)。"""
    h, w = rgb.shape[:2]
    k = size / min(h, w)
    nh, nw = int(round(h * k)), int(round(w * k))
    img = cv2.resize(rgb, (nw, nh), interpolation=cv2.INTER_AREA)
    y0, x0 = max(0, (nh - size) // 2), max(0, (nw - size) // 2)
    return img[y0:y0 + size, x0:x0 + size]


def to_tensor(img):
    return torch.from_numpy(normalize(img).transpose(2, 0, 1)).unsqueeze(0).float()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='0', help='摄像头序号(0/1/..) 或视频文件路径')
    ap.add_argument('--width', type=int, default=640, help='采集宽(默认 640，对齐 OV5640 演示相机)')
    ap.add_argument('--height', type=int, default=480, help='采集高(默认 480)')
    ap.add_argument('--fps', type=int, default=30)
    ap.add_argument('--thresh', type=float, default=0.5, help='检测置信度阈值(默认 0.5)')
    ap.add_argument('--roi', type=float, default=0.55, help='固定 ROI 边长 = 该值×短边(默认 0.55)')
    ap.add_argument('--roi-mode', default='fixed', choices=['fixed', 'auto'],
                    help='fixed(默认)=固定 --roi; auto=用热图扩散微调(注: 实测与手大小相关性 r=-0.02，不算真尺寸估计)')
    ap.add_argument('--roi-c', type=float, default=3.0, help='自适应 ROI: 边长 = roi_c × 热图扩散(仅 auto)')
    ap.add_argument('--roi-min', type=float, default=0.15, help='自适应 ROI 下限(×短边)')
    ap.add_argument('--roi-max', type=float, default=1.0, help='自适应 ROI 上限(×短边)')
    ap.add_argument('--alpha', type=float, default=0.4, help='EMA 平滑系数(默认 0.4)')
    ap.add_argument('--deadzone', type=float, default=5.0, help='死区像素(默认 5px)')
    ap.add_argument('--vote', type=int, default=3, help='手势多数投票帧数(默认 3)')
    ap.add_argument('--device', default=None, help='cuda/cpu，默认自动')
    ap.add_argument('--out', default=str(ROOT / 'results' / 'w2_live'))
    ap.add_argument('--record', default=None, help='录像输出 mp4 路径')
    ap.add_argument('--no-window', action='store_true', help='不开窗口(自检)')
    ap.add_argument('--max-frames', type=int, default=0, help='处理 N 帧后退出(自检)')
    ap.add_argument('--quiet', action='store_true', help='不打印统计(自检)')
    args = ap.parse_args()

    dev = args.device or ('cuda' if torch.cuda.is_available() else 'cpu')
    win = 'hand teleop - live test'
    if not args.no_window:
        try:
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
            cv2.imshow(win, np.zeros((8, 8, 3), np.uint8))
            cv2.waitKey(1)
        except cv2.error as e:
            print('[错误] 当前 opencv 没有 GUI 后端(多半装了 headless 版):', str(e)[:120])
            print('       修法: python -m pip uninstall -y opencv-python-headless')
            print('             python -m pip install opencv-python')
            return 3
    det = DetectNet('softargmax').to(dev).eval()
    det.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'detect_fp32.pt', map_location=dev))
    cls = ClassifyNet().to(dev).eval()
    cls.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'classify_fp32.pt', map_location=dev))
    print(f'[模型] detect + classify(3类) 已加载 | device={dev}')
    print(f'[模型] 权重 SHA256 见 model/frozen/，结构见 docs/网络结构清单.md v1.3')

    src = args.source
    if src.isdigit():
        idx = int(src)
        caps = ([('DSHOW', cv2.CAP_DSHOW), ('MSMF', cv2.CAP_MSMF), ('ANY', cv2.CAP_ANY)]
                if sys.platform.startswith('win') else [('ANY', cv2.CAP_ANY)])
        cap = None
        for name, be in caps:
            c = cv2.VideoCapture(idx, be)
            if c.isOpened():
                ok, _ = c.read()
                if ok:
                    cap = c
                    print(f'[采集] 后端 {name}')
                    break
            c.release()
        if cap is None:
            print(f'[错误] 摄像头 {idx} 打不开（试试 --source 1）')
            return 2
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, args.fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    else:
        cap = cv2.VideoCapture(src)
    if not cap.isOpened():
        print(f'[错误] 打不开视频源: {args.source}')
        return 2
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_src = cap.get(cv2.CAP_PROP_FPS)
    print(f'[采集] {W}x{H} @ {fps_src:.1f}fps (源={args.source})')
    print(f'[参数] 阈值 {args.thresh} | ROI 边长 {args.roi}×短边 | EMA α={args.alpha} | 死区 {args.deadzone}px | 投票 {args.vote} 帧')

    out_dir = Path(args.out)
    (out_dir / 'snapshots').mkdir(parents=True, exist_ok=True)
    painter = make_painter()
    print('[界面] 文字模式:', '中文(PIL)' if painter.cjk else 'ASCII(cv2)')

    writer = None
    if args.record:
        Path(args.record).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(args.record, cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (W, H))

    s = min(W, H)
    cx0, cy0 = (W - s) // 2, (H - s) // 2                      # 检测器可视正方形在原帧中的位置
    ema = None
    votes = deque(maxlen=max(1, args.vote))
    log = {'conf': [], 'cls': Counter(), 'nofire': 0, 'conf_raw': []}
    t_start, n = time.time(), 0
    fps_disp = 0.0
    freeze_on, frozen_frame = False, None
    show_inset = True
    roi_frac = args.roi
    auto_side = None
    thr = args.thresh

    try:
        while True:
            ok, frame_bgr = cap.read()
            if not ok:
                print('[结束] 视频源读完')
                break
            if freeze_on and frozen_frame is not None:
                frame_bgr = frozen_frame
            rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # ---- 检测（同时取热图，用于估计手的大小）----
            det_in = detect_input(rgb)
            with torch.no_grad():
                conf_logit, uv, hm = det(to_tensor(det_in).to(dev), return_hm=True)
                prob = torch.softmax(hm.view(1, -1), -1).view_as(hm)[0, 0]
            conf = float(torch.sigmoid(conf_logit)[0])
            uvn = uv[0].detach().cpu().numpy()
            u, v = float(np.clip(uvn[0], 0, 1)), float(np.clip(uvn[1], 0, 1))
            # 热图二阶矩 = 手的画面占比(在 36 格上)，用它当尺子做尺寸自适应 ROI
            n_hm = prob.shape[0]
            idx = torch.arange(n_hm, dtype=torch.float32, device=prob.device)
            c0, c1 = prob.sum(0), prob.sum(1)
            gu_, gv_ = float((c0 * idx).sum()), float((c1 * idx).sum())
            su = float(torch.sqrt(torch.clamp((c0 * (idx - gu_) ** 2).sum(), min=0)))
            sv = float(torch.sqrt(torch.clamp((c1 * (idx - gv_) ** 2).sum(), min=0)))
            spread = (su + sv) / 2 / n_hm                       # 归一化扩散 0~1
            auto_side = float(np.clip(args.roi_c * spread, args.roi_min, args.roi_max))
            log['conf_raw'].append(conf)
            px, py = cx0 + u * s, cy0 + v * s               # 映射回原帧像素
            if ema is None:
                ema = (px, py)
            else:                                            # EMA + 死区(计划书 §5)
                ex_, ey_ = ema
                if abs(px - ex_) >= args.deadzone or abs(py - ey_) >= args.deadzone:
                    ema = (args.alpha * px + (1 - args.alpha) * ex_,
                           args.alpha * py + (1 - args.alpha) * ey_)
            sx, sy = ema

            # ---- 分类 ----
            cls_id, probs = None, None
            if conf >= thr:
                side = (auto_side if args.roi_mode == 'auto' and auto_side else roi_frac) * s
                bbox = ((px - side / 2) / W, (py - side / 2) / H, side / W, side / H)
                roi = crop_roi(rgb, bbox, 1.5, None, CLS_SIZE, False)
                with torch.no_grad():
                    lg = cls(to_tensor(roi).to(dev))[0]
                    probs = torch.softmax(lg, 0).cpu().numpy()
                cls_id = int(probs.argmax())
                votes.append(cls_id)
                log['cls'][cls_id] += 1
                log['conf'].append(conf)
            else:
                votes.append(-1)
                log['nofire'] += 1

            # ---- 多数投票 -> 夹爪动作 ----
            act, act_col = '夹爪保持 (无动作)', (160, 160, 160)
            if len(votes) >= max(1, args.vote):
                c = Counter(votes).most_common(1)[0]
                if c[0] == 1 and c[1] >= max(1, args.vote):
                    act, act_col = '夹爪 -> 张开', (80, 220, 80)
                elif c[0] == 0 and c[1] >= max(1, args.vote):
                    act, act_col = '夹爪 -> 闭合', (80, 80, 240)

            # ---- 画界面 ----
            vis = frame_bgr.copy()
            cv2.rectangle(vis, (cx0, cy0), (cx0 + s - 1, cy0 + s - 1), (110, 110, 110), 1)
            cv2.putText(vis, 'detector view (144 center crop)', (cx0 + 4, cy0 + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (110, 110, 110), 1, cv2.LINE_AA)
            cv2.drawMarker(vis, (int(px), int(py)), (60, 60, 255), cv2.MARKER_CROSS, 12, 1)
            if conf >= thr:
                cv2.circle(vis, (int(sx), int(sy)), 6, (0, 255, 255), 2)
                sd = (auto_side if args.roi_mode == 'auto' and auto_side else roi_frac) * s * 1.5
                x0, y0 = int(px - sd / 2), int(py - sd / 2)
                cv2.rectangle(vis, (x0, y0), (int(x0 + sd), int(y0 + sd)), (0, 255, 255), 1)
                cv2.putText(vis, 'classifier ROI', (x0, max(12, y0 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1, cv2.LINE_AA)

            texts = []
            y = 8
            texts.append((f'FPS {fps_disp:4.1f}   conf {conf:.3f}   thr {thr:.2f}   '
                          f'ROI {"auto" if args.roi_mode == "auto" else "fixed"} '
                          f'{(auto_side if args.roi_mode == "auto" and auto_side else roi_frac):.2f}×短边',
                          (10, y), (255, 255, 255), 18))
            if conf < thr:
                texts.append(('未检出 / 置信度不足', (10, y + 26), (200, 200, 200), 30))
            else:
                nm, sub = CLS_NAME[cls_id] if painter.cjk else (CLS_ASCII[cls_id], '')
                col = {0: (80, 80, 240), 1: (80, 220, 80), 2: (0, 200, 255)}[cls_id]
                texts.append((nm, (10, y + 24), col, 34))
                texts.append((f'{sub}  p={probs[cls_id]:.2f}', (10, y + 62), (255, 255, 255), 15))
            texts.append((act, (10, y + 92), act_col, 22))
            if painter.cjk:
                texts.append((f'定位 ({sx:.0f},{sy:.0f})px / 帧 {W}x{H}', (10, y + 122), (200, 200, 200), 15))
                texts.append(('q 退出  s 快照  d 小图  [ ] ROI  t/T 阈值  k 清状态  r 录像',
                              (10, H - 10), (180, 180, 180), 14))
            vis = painter.flush(vis, texts)

            # ---- 小图: 模型真正看到的 144 输入 + 分类 ROI ----
            if show_inset:
                big = cv2.resize(cv2.cvtColor(det_in, cv2.COLOR_RGB2BGR), (288, 288),
                                 interpolation=cv2.INTER_NEAREST)
                cv2.rectangle(big, (0, 0), (287, 287), (255, 255, 255), 1)
                cv2.drawMarker(big, (int(u * 288), int(v * 288)), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
                cv2.putText(big, 'model input 144x144', (6, 284), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                vis[0:288, W - 288:W] = big
                if probs is not None:
                    r80 = cv2.resize(cv2.cvtColor(roi, cv2.COLOR_RGB2BGR), (120, 120),
                                     interpolation=cv2.INTER_NEAREST)
                    cv2.rectangle(r80, (0, 0), (119, 119), (0, 255, 255), 1)
                    vis[292:412, W - 288:W - 168] = r80
                    cv2.putText(vis, 'cls ROI 80x80', (W - 288, 292 + 134),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

            if writer is not None:
                writer.write(vis)
            n += 1
            if not args.no_window:
                cv2.imshow(win, vis)
                k = cv2.waitKey(1) & 0xFF
                if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:   # 用 X 关窗也干净退出
                    print('  窗口已关闭')
                    break
                if k in (ord('q'), 27):
                    break
                elif k == ord('s'):
                    ts = time.strftime('%H%M%S')
                    cv2.imwrite(str(out_dir / 'snapshots' / f'frame_{ts}.png'), vis)
                    print(f'  快照 -> snapshots/frame_{ts}.png')
                elif k == ord('d'):
                    show_inset = not show_inset
                elif k == ord('['):
                    roi_frac = max(0.15, roi_frac - 0.05); print(f'  ROI={roi_frac:.2f}')
                elif k == ord(']'):
                    roi_frac = min(0.90, roi_frac + 0.05); print(f'  ROI={roi_frac:.2f}')
                elif k == ord('t'):
                    thr = max(0.05, round(thr - 0.05, 2)); print(f'  阈值={thr:.2f}')
                elif k == ord('T'):
                    thr = min(0.95, round(thr + 0.05, 2)); print(f'  阈值={thr:.2f}')
                elif k == ord('k'):
                    ema = None; votes.clear(); print('  已清空平滑与投票')
                elif k == ord('r'):
                    if writer is None:
                        pth = out_dir / f'rec_{time.strftime("%H%M%S")}.mp4'
                        writer = cv2.VideoWriter(str(pth), cv2.VideoWriter_fourcc(*'mp4v'), args.fps, (W, H))
                        print(f'  开始录像 -> {pth.name}')
                    else:
                        writer.release(); writer = None; print('  停止录像')
                elif k == ord(' '):
                    freeze_on = not freeze_on
                    frozen_frame = vis.copy() if freeze_on else None
                    print('  画面冻结' if freeze_on else '  解除冻结')
            if n % 10 == 0 or n == 1:
                el = time.time() - t_start
                fps_disp = n / el
            if args.max_frames and n >= args.max_frames:
                break
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if not args.no_window:
            cv2.destroyAllWindows()

    el = time.time() - t_start
    summary = {'frames': n, 'seconds': round(el, 2), 'avg_fps': round(n / el, 1) if el else 0,
               'source': str(args.source), 'frame_size': [W, H], 'thresh': thr, 'roi_frac': roi_frac,
               'roi_mode': args.roi_mode, 'roi_c': args.roi_c,
               'n_detect_fire': len(log['conf']), 'n_no_fire': log['nofire'],
               'cls_dist': {int(k): int(v) for k, v in log['cls'].items()},
               'conf_median': float(np.median(log['conf_raw'])) if log['conf_raw'] else None,
               'fire_rate': round(len(log['conf']) / n, 3) if n else 0}
    (out_dir / f'session_{time.strftime("%Y%m%d_%H%M%S")}.json').write_text(
        json.dumps(summary, ensure_ascii=False, indent=1), encoding='utf-8')
    if not args.quiet:
        print(f'\n[统计] 处理 {n} 帧 / {el:.1f}s = {summary["avg_fps"]} fps')
        print(f'  检测触发率 {summary["fire_rate"]*100:.1f}% (conf 中位 {summary["conf_median"]:.3f})')
        print(f'  手势分布 {summary["cls_dist"]}  (0=闭合 1=张开 2=其他)')
        print(f'  摘要 -> {out_dir}/session_*.json')
    return 0


if __name__ == '__main__':
    sys.exit(main())
