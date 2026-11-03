# -*- coding: utf-8 -*-
"""T5：sanity 模型评测 + 可视化 + ONNX 导出（train-pi 环境）。

门槛为交接文档 §3 预登记值（sanity 口径），逐条判 PASS/FAIL，不许事后改。
任务层 5 通道用与 pose_map 完全相同的 2D 公式（theta/palm/hand_px/伸指数），
不经过 PoseMapper（slew/死区会污染逐帧对比）；3D 通道不用教师 world 点，避免污染。
用法：
  D:/Anaconda3/envs/train-pi/python.exe -u tools/eval_kpt.py \
      --cache results/stage1/cache_xxx --run results/stage1/sanity_run1 --video <源视频>
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
sys.path.insert(0, str(TOOLS.parent / 'sw'))
from train_kpt import HandKptNet, HeatmapHead, nme_of   # noqa: E402
import pose_map                                      # noqa: E402  只取 TIP/PIP/PALM_IDX/HAND_LINKS

# 预登记门槛（交接文档 §3，sanity 口径）——改这里等于改规矩，必须先问主人
GATES = {'nme': 0.030, 'pck05': 0.85, 'pck10': 0.95, 'uv': 0.04, 'hp_ratio': 0.10,
         'theta_deg': 6.0, 'ext_match': 0.90, 'jitter_ratio': 1.5, 'ort_ms': 8.0}
MARGIN_RATIO = 0.15


def theta_deg(p):
    return np.degrees(np.arctan2(p[9, 1] - p[0, 1], p[9, 0] - p[0, 0]))


def palm_uv(p):
    return p[pose_map.PALM_IDX].mean(axis=0)


def hand_len(p):
    return float(np.linalg.norm(p[9] - p[0]))


def ext_bits(p):
    hp = hand_len(p)
    m = MARGIN_RATIO * hp
    return tuple(int(np.linalg.norm(p[t] - p[0]) > np.linalg.norm(p[q] - p[0]) + m)
                 for t, q in zip(pose_map.TIP, pose_map.PIP))


def ang_diff(a, b):
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(float(d))


def read_frames(video, idxs):
    want = set(idxs)
    cap = cv2.VideoCapture(str(video))
    got, i = {}, 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        if i in want:
            got[i] = f
            if len(got) == len(want):
                break
        i += 1
    cap.release()
    return got


def tile(frame, pred, tgt, tw, tag):
    H, W = frame.shape[:2]
    th = max(1, int(round(tw * H / W)))
    img = cv2.resize(frame, (tw, th))
    for pts, col in ((tgt, (80, 220, 80)), (pred, (60, 60, 230))):
        if pts is None:
            continue
        px = (np.asarray(pts) * np.array([tw, th], np.float32)).astype(int)
        for a, b in pose_map.HAND_LINKS:
            cv2.line(img, tuple(px[a]), tuple(px[b]), col, 1, cv2.LINE_AA)
        for j in range(21):
            cv2.circle(img, tuple(px[j]), 2, col, -1, cv2.LINE_AA)
    cv2.putText(img, tag, (3, th - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 2)
    cv2.putText(img, tag, (4, th - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    return img


def scatter_panel(canvas, ox, oy, w, h, xs, ys, title, xlim, ylim):
    cv2.rectangle(canvas, (ox, oy), (ox + w, oy + h), (120, 120, 120), 1)
    cv2.line(canvas, (ox, oy + h // 2), (ox + w, oy + h // 2), (70, 70, 70), 1)
    cv2.line(canvas, (ox + w // 2, oy), (ox + w // 2, oy + h), (70, 70, 70), 1)
    cv2.putText(canvas, title, (ox + 4, oy + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                (255, 255, 255), 1)
    for x, y in zip(xs, ys):
        px = int(ox + (x - xlim[0]) / (xlim[1] - xlim[0]) * w)
        py = int(oy + h - (y - ylim[0]) / (ylim[1] - ylim[0]) * h)
        if ox <= px <= ox + w and oy <= py <= oy + h:
            cv2.circle(canvas, (px, py), 2, (90, 200, 255), -1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cache', required=True)
    ap.add_argument('--run', required=True)
    ap.add_argument('--video', required=True)
    args = ap.parse_args()

    cache, run = Path(args.cache), Path(args.run)
    split = json.loads((cache / 'split.json').read_text())
    cache_meta = json.loads((cache / 'meta.json').read_text())
    pts_t = np.load(cache / 'pts.npy')
    pres = np.load(cache / 'presence.npy')
    handc = np.load(cache / 'hand.npy')
    scale = np.load(cache / 'scale.npy')
    roi = np.load(cache / 'roi.npy') if (cache / 'roi.npy').exists() else None
    stage2 = bool(cache_meta.get('stage2', False)) and roi is not None
    test_idx = split['test']
    neg_idx = split.get('neg', [])

    ck = torch.load(run / 'best.pt', map_location='cpu', weights_only=True)
    head = ck.get('config', {}).get('head', 'gap')
    if head == 'heatmap':
        net = HeatmapHead(stride=ck.get('config', {}).get('hm_stride', 8))
    else:
        net = HandKptNet()
    net.load_state_dict(ck['model'])
    net.eval()
    dev = torch.device('cpu')

    def predict(idxs):
        imgs = np.load(cache / 'imgs.npy', mmap_mode='r')
        P = []
        with torch.no_grad():
            for i in idxs:
                x = (np.asarray(imgs[i], np.float32).transpose(2, 0, 1) / 127.5) - 1.0
                c, pl, hl = net(torch.from_numpy(x)[None])
                P.append((c[0].numpy().reshape(21, 2), float(pl[0]), hl[0].numpy()))
        return P

    P_test = predict(test_idx)
    pred = np.stack([p[0] for p in P_test])
    tgt = pts_t[test_idx]
    sc = scale[test_idx]

    # 阶段二缓存：通道指标需仿射回全帧（门槛口径不变）；NME/PCK 留在 ROI 空间（比值不变性近似成立）
    if stage2:
        r = roi[test_idx]
        pred_full = np.stack([r[:, 0][:, None] + pred[..., 0] * r[:, 2][:, None],
                              r[:, 1][:, None] + pred[..., 1] * r[:, 3][:, None]], -1)
        tgt_full = np.stack([r[:, 0][:, None] + tgt[..., 0] * r[:, 2][:, None],
                             r[:, 1][:, None] + tgt[..., 1] * r[:, 3][:, None]], -1)
    else:
        pred_full, tgt_full = pred, tgt

    # ---- 关键点层 ----
    nme = nme_of(pred, tgt, sc)
    err = np.linalg.norm(pred - tgt, axis=2)
    pck05 = float((err < 0.05 * sc[:, None]).mean())
    pck10 = float((err < 0.10 * sc[:, None]).mean())

    # ---- 任务层 5 通道（2D 公式与 pose_map 同源）----
    du, dv, dhp, dth, ext_ok = [], [], [], [], []
    for p, t in zip(pred_full, tgt_full):
        up, vp = palm_uv(p)
        ut, vt = palm_uv(t)
        du.append(abs(up - ut))
        dv.append(abs(vp - vt))
        dhp.append(abs(hand_len(p) - hand_len(t)) / max(hand_len(t), 1e-6))
        dth.append(ang_diff(theta_deg(p), theta_deg(t)))
        ext_ok.append(int(ext_bits(p) == ext_bits(t)))
    uv = float(max(np.mean(du), np.mean(dv)))
    hp_ratio = float(np.mean(dhp))
    theta_err = float(np.mean(dth))
    ext_match = float(np.mean(ext_ok))

    # ---- 抖动：仅时序缓存（视频打标）有定义；i.i.d. 图像缓存相邻帧无关联，显式 SKIP ----
    cache_meta = json.loads((cache / 'meta.json').read_text())
    temporal = bool(cache_meta.get('temporal', True))
    if temporal:
        def jitter(seq):
            d = np.linalg.norm(np.diff(seq, axis=0), axis=2).mean(axis=1)
            return float(d.std())
        j_ratio = jitter(pred) / max(jitter(tgt), 1e-9)
    else:
        j_ratio = None

    # ---- presence / handedness ----
    P_neg = predict(neg_idx) if neg_idx else []
    pres_pred = [p[1] for p in P_test] + [p[1] for p in P_neg]
    pres_gt = [1.0] * len(test_idx) + [0.0] * len(neg_idx)
    pres_acc = float(np.mean([(1 if (a > 0) else 0) == int(b)
                              for a, b in zip(pres_pred, pres_gt)]) ) if pres_pred else 0.0
    hand_pred = [int(p[2].argmax()) for p in P_test]
    hand_gt = [int(h - 1) for h in handc[test_idx]]
    hand_acc = float(np.mean([a == b for a, b in zip(hand_pred, hand_gt)])) if hand_gt else 0.0

    # ---- ONNX 导出 + CPU 延迟 ----
    onnx_path = run / 'best.onnx'
    torch.onnx.export(net, torch.zeros(1, 3, 256, 256), str(onnx_path),
                      input_names=['img'], output_names=['coord', 'presence', 'handed'],
                      dynamo=False, opset_version=17)
    import onnxruntime as ort
    so = ort.SessionOptions()
    so.intra_op_num_threads = 4
    sess = ort.InferenceSession(str(onnx_path), so, providers=['CPUExecutionProvider'])
    x0 = np.zeros((1, 3, 256, 256), np.float32)
    for _ in range(30):
        sess.run(None, {'img': x0})
    t0 = time.time()
    for _ in range(200):
        sess.run(None, {'img': x0})
    ort_ms = (time.time() - t0) / 200 * 1000

    metrics = {'nme': nme, 'pck05': pck05, 'pck10': pck10, 'uv': uv,
               'hp_ratio': hp_ratio, 'theta_deg': theta_err, 'ext_match': ext_match,
               'jitter_ratio': j_ratio, 'ort_ms': ort_ms,
               'presence_acc': pres_acc, 'handedness_acc': hand_acc,
               'n_test': len(test_idx), 'n_neg': len(neg_idx)}
    higher_better = {'pck05', 'pck10', 'ext_match'}
    verdict = {}
    print('\n===== sanity 门槛判定（预登记，交接文档 §3）=====')
    for k, gate in GATES.items():
        v = metrics[k]
        if v is None:
            verdict[k] = {'value': None, 'gate': gate, 'pass': None,
                          'note': 'SKIP：i.i.d. 图像缓存无时序相邻，抖动门槛不适用（M2 视频数据恢复）'}
            print(f"  {k:12s} = {'N/A':>9}   门槛 <= {gate:<7} SKIP（无时序）")
            continue
        ok = v >= gate if k in higher_better else v <= gate
        verdict[k] = {'value': round(float(v), 5), 'gate': gate,
                      'pass': bool(ok)}
        print(f"  {k:12s} = {v:9.4f}   门槛 {'>=' if k in higher_better else '<='} {gate:<7} "
              f"{'PASS' if ok else 'FAIL'}")
    n_fail = sum(1 for v in verdict.values() if v['pass'] is False)
    print(f"  合计 FAIL {n_fail} 项")

    # ---- 可视化 ----
    per_err = err.mean(axis=1)
    np.savez(run / 'test_err.npz', per_err=per_err, scale=sc,
             idx=np.asarray(test_idx), du=np.array(du), dv=np.array(dv),
             dhp=np.array(dhp), dth=np.array(dth), ext_ok=np.array(ext_ok))
    order = np.argsort(per_err)
    picks = list(order[:12]) + list(order[-12:])
    imgs_all = np.load(cache / 'imgs.npy', mmap_mode='r')
    use_video = Path(args.video).exists() and bool(
        json.loads((cache / 'meta.json').read_text()).get('temporal', False))
    frames = read_frames(args.video, test_idx) if use_video else {}

    def frame_of(k):
        if use_video:
            f = frames.get(test_idx[k])
            if f is not None:
                return f
        return cv2.resize(np.asarray(imgs_all[test_idx[k]], np.uint8), (320, 320))
    tiles = []
    for k in picks:
        f = frame_of(k)
        if f is None:
            continue
        kind = 'best' if k < 12 else 'worst'
        tiles.append(tile(f, pred[k], tgt[k], 320,
                          f'#{test_idx[k]} {kind} nme={per_err[k]/max(sc[k],1e-6):.3f}'))
    cols = 6
    rows = (len(tiles) + cols - 1) // cols
    th = tiles[0].shape[0]
    canvas = np.zeros((rows * th, cols * 320, 3), np.uint8)
    for k, t in enumerate(tiles):
        canvas[(k // cols) * th:(k // cols + 1) * th, (k % cols) * 320:(k % cols + 1) * 320] = t
    cv2.imwrite(str(run / 'vis_grid.png'), canvas)

    seq = test_idx[:60]
    st = []
    for k, fi in enumerate(seq):
        if use_video:
            f = frames.get(fi)
        else:
            f = cv2.resize(np.asarray(imgs_all[fi], np.uint8), (240, 240))
        if f is None:
            continue
        st.append(tile(f, pred[k], tgt[k], 240,
                       f'T{theta_deg(tgt[k]):+.0f}/{sum(ext_bits(tgt[k]))} '
                       f'P{theta_deg(pred[k]):+.0f}/{sum(ext_bits(pred[k]))}'))
    cols2 = 10
    rows2 = (len(st) + cols2 - 1) // cols2
    th2 = st[0].shape[0]
    canvas2 = np.zeros((rows2 * th2, cols2 * 240, 3), np.uint8)
    for k, t in enumerate(st):
        canvas2[(k // cols2) * th2:(k // cols2 + 1) * th2,
                (k % cols2) * 240:(k % cols2 + 1) * 240] = t
    cv2.imwrite(str(run / 'vis_seq.png'), canvas2)

    chan = np.zeros((460, 900, 3), np.uint8)
    ut = np.array([palm_uv(t)[0] for t in tgt])
    up = np.array([palm_uv(p)[0] for p in pred])
    vt = np.array([palm_uv(t)[1] for t in tgt])
    vp = np.array([palm_uv(p)[1] for p in pred])
    tt = np.array([theta_deg(t) for t in tgt])
    tp = np.array([theta_deg(p) for p in pred])
    ht = np.array([hand_len(t) for t in tgt])
    hpp = np.array([hand_len(p) for p in pred])
    scatter_panel(chan, 20, 20, 400, 190, ut, up, 'u: teacher x pred', (0, 1), (0, 1))
    scatter_panel(chan, 460, 20, 400, 190, vt, vp, 'v: teacher x pred', (0, 1), (0, 1))
    scatter_panel(chan, 20, 240, 400, 190, tt, tp, 'theta deg', (-180, 180), (-180, 180))
    scatter_panel(chan, 460, 240, 400, 190, ht, hpp, 'hand_len', (0, 0.4), (0, 0.4))
    cv2.imwrite(str(run / 'vis_chan.png'), chan)

    (run / 'eval.json').write_text(json.dumps(
        {'metrics': {k: (round(float(v), 5) if v is not None else None)
                     for k, v in metrics.items()},
         'verdict': verdict, 'n_fail': n_fail,
         'created': time.strftime('%Y-%m-%d %H:%M:%S')},
        ensure_ascii=False, indent=1), encoding='utf-8')
    print(f'[可视化] {run}/vis_grid.png vis_seq.png vis_chan.png（红=预测 绿=教师）')
    print('[完成] eval.json 已写。把三张图贴给主人。')


if __name__ == '__main__':
    main()
