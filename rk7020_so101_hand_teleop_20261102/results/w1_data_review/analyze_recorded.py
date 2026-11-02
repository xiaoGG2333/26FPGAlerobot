"""自录视频抽帧后的数据审查（只读，不改任何数据）。

看四件事：
  1. 光照真的变了吗？     → 每帧平均亮度（亮度曲线）
  2. 真的运动模糊吗？     → 每帧 Laplacian 方差（锐度曲线，统一缩放到长边 640 再比）
  3. 画面里有没有目标手？ → 用**冻结的检测网**跑部署口径（短边 144 + 中心裁剪）的 conf/u/v
  4. 手势状态对不对？     → 用**冻结的分类网**做粗略探测（ROI 取预测中心附近 0.35×短边），
                            拿文件名当标签对照：open→palm(1)、fist→fist(0)

跑法（工程根）:
    D:/Anaconda3/envs/fpga_amd-pi/python.exe results/w1_data_review/analyze_recorded.py
"""
import json
import random
import sys
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / 'model'
OUT = Path(__file__).resolve().parent
FRAMES = ROOT / 'model' / 'data' / 'recorded_neg' / 'frames'
sys.path.insert(0, str(MODEL))
from detect.net import DetectNet                          # noqa: E402
from classify.net import ClassifyNet                       # noqa: E402
from classify.dataset import crop_roi                      # noqa: E402
from common.transforms import load_jpeg_geom, normalize    # noqa: E402

SIZE = 144


def probe_detect(net, path, device):
    """按部署口径（短边缩到 144 + 中心裁剪）跑检测网。"""
    img = load_jpeg_geom(path, SIZE, False, random.Random(0))[0]
    x = torch.from_numpy(normalize(img).transpose(2, 0, 1)).unsqueeze(0).to(device)
    with torch.no_grad():
        p = net.predict(x)[0].cpu().numpy()
    return float(p[0]), float(p[1]), float(p[2])


def probe_classify(net, frame_bgr, cu, cv_, device):
    """在预测中心附近取 ROI（0.35×短边）→ 分类网。粗探，仅用于对照手势状态。"""
    h, w = frame_bgr.shape[:2]
    s = min(h, w)
    x = (w - s) / 2 + cu * s          # 中心裁剪坐标 → 原帧像素
    y = (h - s) / 2 + cv_ * s
    side = 0.35 * s
    bbox = ((x - side / 2) / w, (y - side / 2) / h, side / w, side / h)
    roi = crop_roi(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB), bbox, 1.5,
                   random.Random(0), 80, False)
    x_t = torch.from_numpy(normalize(roi).transpose(2, 0, 1)).unsqueeze(0).to(device)
    with torch.no_grad():
        logit = net(x_t)[0]
        prob = torch.softmax(logit, 0).cpu().numpy()
    return int(prob.argmax()), float(prob.max())      # 0=fist, 1=palm


def sharp(gray):
    g = cv2.resize(gray, (640, 640), interpolation=cv2.INTER_AREA)
    return float(cv2.Laplacian(g, cv2.CV_64F).var())


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    det = DetectNet().to(device).eval()
    det.load_state_dict(torch.load(MODEL / 'frozen' / 'detect_fp32.pt', map_location=device))
    cls = ClassifyNet().to(device).eval()
    cls.load_state_dict(torch.load(MODEL / 'frozen' / 'classify_fp32.pt', map_location=device))

    clips = sorted(d for d in FRAMES.iterdir() if d.is_dir())
    rep, sheets = {}, []
    for clip in clips:
        files = sorted(clip.glob('*.jpg'))
        rec = {'n': len(files), 'brightness': [], 'sharpness': [], 'conf': [], 'uv': [],
               'cls_pred': [], 'cls_prob': []}
        for f in files:
            frame = cv2.imread(str(f))
            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            conf, u, v = probe_detect(det, f, device)
            rec['brightness'].append(round(float(gray.mean()), 1))
            rec['sharpness'].append(round(sharp(gray), 1))
            rec['conf'].append(round(conf, 3))
            rec['uv'].append([round(u, 3), round(v, 3)])
            if conf > 0.7:
                c, p = probe_classify(cls, frame, u, v, device)
                rec['cls_pred'].append(c); rec['cls_prob'].append(round(p, 3))
        b, sh, cf = map(np.array, (rec['brightness'], rec['sharpness'], rec['conf']))
        pred = np.array(rec['cls_pred'])
        rec['shape'] = [w, h]
        rec['summary'] = {
            'brightness_min_max_std': [float(b.min()), float(b.max()), round(float(b.std()), 1)],
            'sharpness_min_med_max': [float(sh.min()), float(np.median(sh)), float(sh.max())],
            'conf_med_min': [round(float(np.median(cf)), 3), round(float(cf.min()), 3)],
            'share_conf_gt_0.7': round(float((cf > 0.7).mean()), 3),
            'cls_palm_share': round(float((pred == 1).mean()), 3) if len(pred) else None,
            'cls_prob_med': round(float(np.median(rec['cls_prob'])), 3) if pred.size else None,
        }
        rep[clip.name] = rec
        print(f"{clip.name:18s} {w}x{h} n={len(files):3d} "
              f"亮度 {b.min():5.0f}~{b.max():5.0f}(σ{b.std():4.1f}) "
              f"锐度 中位{np.median(sh):7.1f} 最小{sh.min():6.1f} "
              f"conf 中位{np.median(cf):.3f} 最低{cf.min():.3f} >0.7占比{(cf>0.7).mean():.2f} "
              f"分类→palm占比 {rec['summary']['cls_palm_share']}")

        # 抽 6 帧做联系表，供人看
        idxs = np.linspace(0, len(files) - 1, 6).astype(int)
        tiles = []
        for i in idxs:
            im = cv2.imread(str(files[i]))
            hh = 300
            im = cv2.resize(im, (int(im.shape[1] * hh / im.shape[0]), hh))
            lab = f"f{i+1} B{rec['brightness'][i]:.0f} S{rec['sharpness'][i]:.0f} c{rec['conf'][i]:.2f}"
            cv2.putText(im, lab, (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
            tiles.append(im)
        sheet = np.hstack(tiles)
        path = OUT / f'sheet_{clip.name}.png'
        cv2.imwrite(str(path), sheet)
        sheets.append(str(path))
    (OUT / 'recorded_review.json').write_text(json.dumps(rep, indent=1, ensure_ascii=False),
                                              encoding='utf-8')
    print('\n联系表:', *sheets, sep='\n  ')
    print('报告:', OUT / 'recorded_review.json')


if __name__ == '__main__':
    main()


def render_deploy_crops():
    """额外渲染：每段抽 6 帧的**部署口径中心裁剪（144x144）**，4 段拼一张图。
    用来直接回答：演示裁剪窗里到底有没有手。"""
    import sys as _sys
    _sys.path.insert(0, str(MODEL))
    clips = sorted(d for d in FRAMES.iterdir() if d.is_dir())
    rows = []
    for clip in clips:
        files = sorted(clip.glob('*.jpg'))
        idxs = np.linspace(0, len(files) - 1, 6).astype(int)
        tiles = []
        for i in idxs:
            img = load_jpeg_geom(files[i], SIZE, False, random.Random(0))[0]
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)   # 管线内部是 RGB，imwrite 要 BGR
            t = cv2.resize(img, (192, 192), interpolation=cv2.INTER_NEAREST)
            cv2.putText(t, f"{clip.name[:9]} f{i+1}", (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            tiles.append(t)
        rows.append(np.hstack(tiles))
    out = OUT / 'sheet_deploy_crops.png'
    cv2.imwrite(str(out), np.vstack(rows))
    print('部署裁剪窗联系表:', out)


if __name__ == '__main__':
    render_deploy_crops()
