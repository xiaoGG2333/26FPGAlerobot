// 阶段1 手部关键点网 —— IP 功能模型实现（与 tools/int8_ref.py 等价）
//
// 说明：这里用堆上的 vector 做缓冲，是"算法正确性"版本；
// HLS 版会换成带 pragma 的静态数组 + 按 band 流式复用，算法体不变。
#include "kpt_net.h"

#include <vector>
#include <cstdio>

namespace kpt {

void conv_layer(const KptLayer& L, const Params& P, const uint8_t* x, int hin, int win,
                uint8_t* y, int hout, int wout) {
  const int cin = L.cin, cout = L.cout;
  const int grp = (L.kind == 2) ? cin : 1;   // dw 层 group == cin
  const int cg = cin / grp;
  const int og = cout / grp;
  const int k = (L.kind == 1) ? 1 : 3;
  const int s = L.stride;
  const int8_t* w = P.w + L.w_off;
  const int32_t* b = P.b + L.b_off;
  const int32_t* mult = P.mult + L.m_off;
  const int8_t* sh = P.sh + L.s_off;
  const int xzp = L.x_zp;

  for (int g = 0; g < grp; ++g) {
    for (int o = 0; o < og; ++o) {
      const int co = g * og + o;
      const int8_t* wg = w + (int64_t)co * cg * k * k;
      for (int oy = 0; oy < hout; ++oy) {
        for (int ox = 0; ox < wout; ++ox) {
          int64_t acc = 0;
          for (int ic = 0; ic < cg; ++ic) {
            const int ci = g * cg + ic;
            const int8_t* wp = wg + (int64_t)ic * k * k;
            for (int ky = 0; ky < k; ++ky) {
              const int iy = oy * s - L.pad_t + ky;
              if (iy < 0 || iy >= hin) continue;   // 量化域零填充 = x_zp，贡献为 0
              for (int kx = 0; kx < k; ++kx) {
                const int ix = ox * s - L.pad_l + kx;
                if (ix < 0 || ix >= win) continue;
                acc += (int64_t)((int)x[(int64_t)ci * hin * win + (int64_t)iy * win + ix] - xzp) *
                       (int64_t)wp[ky * k + kx];
              }
            }
          }
          acc += (int64_t)b[co];
          y[(int64_t)co * hout * wout + (int64_t)oy * wout + ox] =
              requant(acc, mult[co], sh[co], L.y_zp);
        }
      }
    }
  }
}

void add_layer(const KptLayer& L, const uint8_t* a, const uint8_t* b, uint8_t* y, int n) {
  // out = ((a - a_zp)*ka + (b - b_zp)*kb) + y_zp，ka/kb 已折成 Q{L.qsh} 定点系数
  const int S = L.qsh;
  const int64_t rnd = (int64_t)1 << (S - 1);
  for (int i = 0; i < n; ++i) {
    int64_t v = (int64_t)((int)a[i] - L.a_zp) * L.am + (int64_t)((int)b[i] - L.b_zp) * L.bm;
    int64_t r = (v >= 0) ? ((v + rnd) >> S) : -(((-v) + rnd) >> S);
    int q = (int)(r + L.y_zp);
    if (q < 0) q = 0;
    if (q > 255) q = 255;
    y[i] = (uint8_t)q;
  }
}

void gap_layer(const KptLayer& L, const uint8_t* x, int c, int h, int w, uint8_t* y) {
  // am 里已经把除以 (h*w) 折进去了；全域用整数，避免 double 在 HLS 里吃 DSP
  const int S = L.qsh;
  const int64_t rnd = (int64_t)1 << (S - 1);
  for (int i = 0; i < c; ++i) {
    int64_t acc = 0;
    for (int j = 0; j < h * w; ++j) acc += (int)x[(int64_t)i * h * w + j] - L.x_zp;
    int64_t v = acc * L.am;
    int64_t r = (v >= 0) ? ((v + rnd) >> S) : -(((-v) + rnd) >> S);
    int q = (int)(r + L.y_zp);
    if (q < 0) q = 0;
    if (q > 255) q = 255;
    y[i] = (uint8_t)q;
  }
}

void fc_layer(const KptLayer& L, const Params& P, const uint8_t* x, uint8_t* y) {
  const int8_t* w = P.w + L.w_off;
  const int32_t* b = P.b + L.b_off;
  const int32_t* mult = P.mult + L.m_off;
  const int8_t* sh = P.sh + L.s_off;
  for (int o = 0; o < L.cout; ++o) {
    int64_t acc = 0;
    for (int i = 0; i < L.cin; ++i)
      acc += (int64_t)((int)x[i] - L.x_zp) * (int64_t)w[(int64_t)o * L.cin + i];
    acc += (int64_t)b[o];
    y[o] = requant(acc, mult[o], sh[o], L.y_zp);
  }
}

static void trace(int li, const KptLayer& L, const std::vector<uint8_t>& b,
                  int c, int h, int w) {
#ifdef KPT_TRACE
  int mn = 255, mx = 0;
  long long sum = 0;
  for (size_t i = 0; i < b.size(); ++i) {
    int v = b[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
  }
  printf("  L%02d k%d %dx%dx%d  [%d,%d] mean %.1f\n", li, L.kind, c, h, w, mn, mx,
         b.empty() ? 0.0 : (double)sum / (double)b.size());
#else
  (void)li; (void)L; (void)b; (void)c; (void)h; (void)w;
#endif
}

void forward(const KptLayer* layers, int n_layers, const Params& P,
             const uint8_t* img, uint8_t* hm, uint8_t* pres, uint8_t* hand) {
  // 输入是 HWC（相机像素顺序），内部一律 CHW
  std::vector<uint8_t> cur((int64_t)3 * IN_SIZE * IN_SIZE);
  for (int y = 0; y < IN_SIZE; ++y)
    for (int x = 0; x < IN_SIZE; ++x)
      for (int ch2 = 0; ch2 < 3; ++ch2)
        cur[(int64_t)ch2 * IN_SIZE * IN_SIZE + (int64_t)y * IN_SIZE + x] =
            img[((int64_t)y * IN_SIZE + x) * 3 + ch2];
  std::vector<uint8_t> nxt, skip;
  int c = 3, h = IN_SIZE, w = IN_SIZE;
  bool have_hm = false;
  std::vector<uint8_t> fc_out[3];
  std::vector<uint8_t> fc_in;    // 256 维公共输入（pres/hand 两个头是并列的，不是串联）
  int n_fc = 0;

  for (int li = 0; li < n_layers; ++li) {
    const KptLayer& L = layers[li];
    if (L.kind == 0 || L.kind == 1 || L.kind == 2) {
      // 进入新 block（扩张层 cout > cin）时保存跳跃支路
      if (L.kind == 1 && L.cout > L.cin) skip = cur;
      const int k = (L.kind == 1) ? 1 : 3;
      const int s = L.stride;
      const int hout = (h + L.pad_t + L.pad_b - k) / s + 1;
      const int wout = (w + L.pad_l + L.pad_r - k) / s + 1;
      trace(li, L, cur, c, h, w);
      if (L.cout == N_KPT) {                 // 热图支路：吃 feat.13，不改主干的 cur
        nxt.assign((int64_t)L.cout * hout * wout, 0);
        conv_layer(L, P, cur.data(), h, w, nxt.data(), hout, wout);
        memcpy(hm, nxt.data(), (int64_t)L.cout * hout * wout);
        have_hm = true;
      } else {
        nxt.assign((int64_t)L.cout * hout * wout, 0);
        conv_layer(L, P, cur.data(), h, w, nxt.data(), hout, wout);
        cur.swap(nxt);
        c = L.cout;
        h = hout;
        w = wout;
      }
    } else if (L.kind == 3) {   // 残差
      nxt.assign(cur.size(), 0);
      // ONNX 里 Add 的 a 是跳跃支路、b 是 block 投影输出（层表 x_scale=a_scale）
      add_layer(L, skip.data(), cur.data(), nxt.data(), (int)cur.size());
      cur.swap(nxt);
    } else if (L.kind == 4) {   // GAP
      nxt.assign(c, 0);
      gap_layer(L, cur.data(), c, h, w, nxt.data());
      cur.swap(nxt);
      h = w = 1;
      trace(li, L, cur, c, h, w);
    } else if (L.kind == 5) {   // FC
      if (L.cout == 256) {      // 隐藏层，产生 pres/hand 共用的输入
        nxt.assign(L.cout, 0);
        fc_layer(L, P, cur.data(), nxt.data());
        fc_in = nxt;
      } else {                  // pres(1) 或 hand(2)：都从 fc_in 出发
        nxt.assign(L.cout, 0);
        fc_layer(L, P, fc_in.data(), nxt.data());
      }
      cur.swap(nxt);
      c = L.cout;
      trace(li, L, cur, c, h, w);
      h = w = 1;
      if (n_fc < 3) fc_out[n_fc++] = cur;
    }
  }
  (void)have_hm;
  if (n_fc == 3) {
    pres[0] = fc_out[1][0];
    hand[0] = fc_out[2][0];
    hand[1] = fc_out[2][1];
  }
}

}  // namespace kpt
