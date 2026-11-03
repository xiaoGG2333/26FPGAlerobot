# -*- coding: utf-8 -*-
"""C++ 功能模型 vs onnxruntime int8 金标准：逐张比热图 logits。

这是"上 HLS 之前必须过"的那道门：C 代码算的定点结果和金标准差超过 2 LSB，
就说明定点语义还没对齐，别急着综合。

用法（先编译 C++）：
  cd fpga/ip/kpt_net && mkdir -p build && \
    g++ -O2 -std=c++17 -I data -I src tb/tb_kpt_net.cpp src/kpt_net.cpp -o build/tb_kpt_net.exe
  D:/Anaconda3/envs/train-pi/python.exe -u tools/check_cpp_ip.py \
      --onnx results/stage1/self_run5_192/int8/best_hm_int8.onnx \
      --cache results/stage1/cache_cam480_v3 --n 6
"""
import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import int8_ref as R  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--onnx', default='results/stage1/self_run5_192/int8/best_hm_int8.onnx')
    ap.add_argument('--cache', default='results/stage1/cache_cam480_v3')
    ap.add_argument('--ip', default='fpga/ip/kpt_net')
    ap.add_argument('--n', type=int, default=6)
    ap.add_argument('--stride', type=int, default=60)
    args = ap.parse_args()

    ip = ROOT / args.ip
    tb = ip / 'build' / 'tb_kpt_net.exe'
    if not tb.exists():
        raise SystemExit(f'先编译 tb：{tb} 不存在')
    gg = R.G(ROOT / args.onnx)
    plan = R.build_plan(gg)
    last = [L for L in plan if L['op'] == 'conv'][-1]
    ys, yz = last['y_scale'], last['y_zp']
    fcs = [L for L in plan if L['op'] == 'fc']
    psy, psz = fcs[1]['y_scale'], fcs[1]['y_zp']
    hsy, hsz = fcs[2]['y_scale'], fcs[2]['y_zp']

    so = ort.SessionOptions()
    so.log_severity_level = 3
    sess = ort.InferenceSession(str(ROOT / args.onnx), so, providers=['CPUExecutionProvider'])
    imgs = np.load(ROOT / args.cache / 'imgs.npy', mmap_mode='r')
    tmp = ip / 'build'
    diffs = []
    for k in range(args.n):
        im = cv2.resize(np.asarray(imgs[k * args.stride]), (192, 192),
                        interpolation=cv2.INTER_AREA)
        (tmp / 'input.bin').write_bytes(im.tobytes())
        xf = im.astype(np.float32) / 127.5 - 1.0
        o = sess.run(None, {'img': xf.transpose(2, 0, 1)[None]})
        ref = np.clip(np.rint(o[0][0] / ys) + yz, 0, 255).astype(np.uint8)
        r = subprocess.run([str(tb), 'data/kpt_weights.bin', str(tmp / 'input.bin'),
                            str(tmp / 'out.bin')], cwd=str(ip),
                           capture_output=True, text=True)
        if r.returncode:
            raise SystemExit(f'tb 运行失败：{r.stdout} {r.stderr}')
        out = np.fromfile(tmp / 'out.bin', np.uint8)
        hm = out[:21 * 144].reshape(21, 12, 12)
        pres, hand = int(out[21 * 144]), list(out[21 * 144 + 1:])
        rp = np.clip(np.rint(np.asarray(o[1]).ravel()[0] / psy) + psz, 0, 255)
        rh = np.clip(np.rint(np.asarray(o[2]).ravel() / hsy) + hsz, 0, 255)
        d = np.abs(hm.astype(np.int16) - ref.astype(np.int16))
        diffs.append(float(d.mean()))
        print(f'  #{k*args.stride:4d}  hm |diff| 平均 {d.mean():5.2f}  最大 {d.max():4d}  '
              f'超 2 LSB {(d>2).mean()*100:5.1f}%   pres {pres}/{int(rp)} hand {hand}/{list(rh.astype(int))}')
    print(f'\n[结论] C++ 定点模型 vs ORT int8：平均 |diff| {np.mean(diffs):.2f} LSB '
          f'（{"通过" if np.mean(diffs) <= 1.0 else "未通过"}，门槛 <= 1.0）')


if __name__ == '__main__':
    main()
