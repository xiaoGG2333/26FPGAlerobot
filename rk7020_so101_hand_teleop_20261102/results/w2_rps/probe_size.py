"""能否用检测器已有的热图扩散程度估计手的大小？(若行 -> 部署端可做"尺寸自适应 ROI"，不用重训)

测量: 在 RPS(白底, 手可精确分割得到真实手框) 上
  A. 真实手大小(占短边比例) vs 热图二阶矩(扩散) 的相关系数
  B. 用 热图扩散 当尺子构造 ROI = c*spread 时，三类分类准确率随 c 的变化
     (对照: 当前部署固定 ROI=0.35 -> 47.6%)
"""
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'model'))
from common.transforms import normalize      # noqa: E402
from detect.net import DetectNet             # noqa: E402
from classify.net import ClassifyNet         # noqa: E402
from classify.dataset import crop_roi        # noqa: E402

CLS = {'paper': 1, 'rock': 0, 'scissors': 2}
NAME = {0: '闭合', 1: '张开', 2: '其他'}


def hand_box(rgb):
    """白底 -> 非白像素最大连通域 = 手框（RPS 专用）"""
    g = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    m = (g < 225).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m, 8)
    if n < 2:
        return None
    i = 1 + int(np.argmax(stats[1:, 4]))
    return stats[i][:4]


def det_input(rgb, size=144):
    h, w = rgb.shape[:2]
    k = size / min(h, w)
    im = cv2.resize(rgb, (int(round(w * k)), int(round(h * k))), interpolation=cv2.INTER_AREA)
    y0, x0 = max(0, (im.shape[0] - size) // 2), max(0, (im.shape[1] - size) // 2)
    return im[y0:y0 + size, x0:x0 + size]


def stats_of(hm):
    """hm: (N,N) 概率图 -> 期望位置 + 两轴二阶矩(格)"""
    p = hm / max(hm.sum(), 1e-9)
    n = hm.shape[0]
    idx = np.arange(n, dtype=np.float64)
    c0, c1 = p.sum(0), p.sum(1)
    gu = float((c0 * idx).sum())
    gv = float((c1 * idx).sum())
    su = float(np.sqrt(max((c0 * (idx - gu) ** 2).sum(), 0)))
    sv = float(np.sqrt(max((c1 * (idx - gv) ** 2).sum(), 0)))
    return gu, gv, su, sv


def main():
    dev = 'cuda' if torch.cuda.is_available() else 'cpu'
    det = DetectNet('softargmax').to(dev).eval()
    det.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'detect_fp32.pt', map_location=dev))
    cls = ClassifyNet().to(dev).eval()
    cls.load_state_dict(torch.load(ROOT / 'model' / 'frozen' / 'classify_fp32.pt', map_location=dev))

    files = [(p, c) for c in ('paper', 'rock', 'scissors')
             for p in sorted((ROOT / 'model' / 'data' / 'external' / 'rps' / 'test' / c).glob('*.png'))]
    cache = []
    for p, c in files:
        rgb = cv2.cvtColor(cv2.imread(str(p)), cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        s = min(h, w)
        hb = hand_box(rgb)
        x = torch.from_numpy(normalize(det_input(rgb)).transpose(2, 0, 1)).unsqueeze(0).float().to(dev)
        with torch.no_grad():
            conf, uv, hm = det(x, return_hm=True)
        hm_p = torch.softmax(hm.view(1, -1), -1).view_as(hm)[0, 0].cpu().numpy()   # 注意: hy 返回的是 logits
        _, _, su, sv = stats_of(hm_p)
        cache.append({'rgb': rgb, 's': s, 'w': w, 'h': h, 'gt': CLS[c], 'cls': c,
                      'truth': (float(max(hb[2], hb[3])) / s) if hb is not None else None,
                      'spread': (su + sv) / 2, 'uv': (float(uv[0, 0]), float(uv[0, 1])),
                      'pmax': float(hm_p.max()), 'ent': float(-(hm_p * np.log2(np.maximum(hm_p, 1e-12))).sum()),
                      'conf': float(torch.sigmoid(conf)[0])})

    tr = np.array([d['truth'] for d in cache if d['truth']])
    sp = np.array([d['spread'] for d in cache if d['truth']])
    r = np.corrcoef(tr, sp)[0, 1]
    rank = np.corrcoef(np.argsort(np.argsort(tr)), np.argsort(np.argsort(sp)))[0, 1]
    print(f'[A] 真实手大小 vs 热图扩散: Pearson r={r:.3f}  Spearman={rank:.3f}  n={len(tr)}')
    print(f'    手占短边 中位 {np.median(tr)*100:.0f}% | 热图二阶矩 中位 {np.median(sp):.1f}/36 格')
    print(f'    热图: 峰值概率 {np.median([d["pmax"] for d in cache]):.3f} | 熵 中位 {np.median([d["ent"] for d in cache]):.2f}/{np.log2(1296):.2f} bit')
    print(f'    参考: conf 与手大小 r={np.corrcoef(tr, np.array([d["conf"] for d in cache if d["truth"]]))[0,1]:.3f}')

    print('\n[B] 用热图扩散当尺子: ROI 边长 = c × 扩散(格)/36 × 短边, 上限=满幅')
    print('    c        总准确率   闭合    张开    其他   其他误动率')
    for c in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 5.0):
        ok, sup, cm = Counter(), Counter(), defaultdict(Counter)
        for d in cache:
            side = min(d['s'], c * d['spread'] / 36.0 * d['s'])
            px = (d['w'] - d['s']) / 2 + d['uv'][0] * d['s']
            py = (d['h'] - d['s']) / 2 + d['uv'][1] * d['s']
            bbox = ((px - side / 2) / d['w'], (py - side / 2) / d['h'], side / d['w'], side / d['h'])
            roi = crop_roi(d['rgb'], bbox, 1.5, None, 80, False)
            xt = torch.from_numpy(normalize(roi).transpose(2, 0, 1)).unsqueeze(0).float().to(dev)
            with torch.no_grad():
                q = int(cls(xt)[0].argmax())
            g = d['gt']
            sup[g] += 1
            ok[g] += int(q == g)
            cm[g][q] += 1
        n = sum(sup.values())
        om = (cm[2][0] + cm[2][1]) / max(1, sup[2])
        print(f'   {c:4.1f}     {sum(ok.values())/n*100:6.1f}%  {ok[0]/sup[0]*100:6.1f}% '
              f'{ok[1]/sup[1]*100:6.1f}% {ok[2]/sup[2]*100:6.1f}%   {om*100:6.1f}%')
    print('\n    对照 当前部署(固定 ROI=0.35):      47.6%    38.7%  100.0%    4.0%      96.0%')
    print('    对照 完美手框(A):                  68.3%    86.3%   71.8%   46.8%        -')


if __name__ == '__main__':
    main()
