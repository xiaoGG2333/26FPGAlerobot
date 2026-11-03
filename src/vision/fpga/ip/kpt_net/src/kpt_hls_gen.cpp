// 阶段1 手部关键点网 —— 方案 B：逐层展开的直写调度（V3）
//
// 与 kpt_hls.cpp（V2.6，层表驱动）的区别：
//   V2.6 顶层一次调用 forward_hls，里面是一层 for 循环 + 5 路动态分派，
//         所有循环边界都是运行期变量（cin/cout/stride 从层表读）。
//   V3   整个网由 tools/gen_ip_header.py 生成成 52 行显式调用
//         （data/kpt_net_gen.h），每一层的 cin/cout/stride/pad/激活偏移
//         全都是字面量，HLS 拿到的每个循环都有静态 trip count。
//
// 为什么需要这个版本：V2.6 综合时 HLS 在"52 层全内联 + 动态边界"的巨型 CFG 上
// 做 cdfg-build，20 多分钟不出结果。静态边界后每一层都是可以独立流水化的循环。
//
// 定点语义与 kpt_hls.cpp / kpt_net.cpp 完全一致，必须逐位对拍（tb_kpt_hls.cpp）。
//
// g++ 编译时加 -Wno-unknown-pragmas。
#include <cstdio>

#include "kpt_net.h"

namespace kptg {

using kpt::IN_SIZE;
using kpt::HM;
using kpt::N_KPT;
using kpt::requant;

// ---- 片上块 ----------------------------------------------------------------
#ifndef NMAC
#define NMAC 64
#endif
#ifndef NCH
#define NCH 32
#endif
#define NLANE 32
#define CIN_MAX 512

static int8_t  WT[NMAC][CIN_MAX];
static int32_t BT[NMAC];
static int32_t MT[NMAC];
static int8_t  ST[NMAC];
static int8_t  WD[NCH][3][3];

static uint8_t FC_IN[256];
static uint8_t FC_HID[256];

// 三路输出（外部可见，供 testbench 取数）
uint8_t gen_HM_BUF[N_KPT * HM * HM];
uint8_t gen_FC_P[1];
uint8_t gen_FC_H[2];

// 小端读 int32（HLS 不允许在 m_axi 指针上 reinterpret_cast）
static inline int32_t rd32(const uint8_t* p, int off) {
  uint32_t v = (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) |
               ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
  return (int32_t)v;
}
// boff/moff/soff 都是【各自段内的索引】（功能模型写的是 P.b + L.b_off 这种）
static inline int32_t bias_at(const uint8_t* w, int i) {
  return rd32(w, KPT_OFF_BIAS + 4 * i);
}
static inline int32_t mult_at(const uint8_t* w, int i) {
  return rd32(w, KPT_OFF_MULT + 4 * i);
}
static inline int8_t sh_at(const uint8_t* w, int i) {
  return (int8_t)w[KPT_OFF_SHIFT + i];
}

// ---------------------------------------------------------------------------
// 逐点 1x1
// ---------------------------------------------------------------------------
static void k_pw(const uint8_t* w, uint8_t* sc, int xb, int yb,
                 int cin, int cout, int xzp, int yzp,
                 int woff, int boff, int moff, int soff, int hw) {
#pragma HLS ARRAY_PARTITION variable=WT complete dim=1
#pragma HLS ARRAY_PARTITION variable=BT complete
#pragma HLS ARRAY_PARTITION variable=MT complete
#pragma HLS ARRAY_PARTITION variable=ST complete
  // 方案 B 的坑：cout 变成字面量后，HLS 认为「2~7 次的分块循环太小」就直接整层展开，
  // 代码量炸开 -> XFORM 203-504 主动中止。把分块次数放进 volatile，
  // 让编译器拿不到 trip count，就不会自动展开。
  volatile int nblk_v = (cout + NMAC - 1) / NMAC;
  const int nblk = nblk_v;
  for (int b = 0; b < nblk; ++b) {
    const int co0 = b * NMAC;
    const int cn = (cout - co0 < NMAC) ? (cout - co0) : NMAC;
    for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
      const int ok = (j < cn) ? 1 : 0;
      BT[j] = ok ? bias_at(w, boff + co0 + j) : 0;
      MT[j] = ok ? mult_at(w, moff + co0 + j) : 0;
      ST[j] = ok ? sh_at(w, soff + co0 + j) : (int8_t)0;
      for (int ci = 0; ci < cin; ++ci) {
#pragma HLS PIPELINE II=1
        WT[j][ci] = ok ? (int8_t)w[woff + (co0 + j) * cin + ci] : (int8_t)0;
      }
    }
    for (int p = 0; p < hw; ++p) {
#pragma HLS PIPELINE II=1
      int32_t acc[NMAC];
#pragma HLS ARRAY_PARTITION variable=acc complete
      for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
        acc[j] = BT[j];
      }
      volatile int cin_v = cin;      // opaque：否则 cin 字面量会让 HLS 整层展开 288 次累加
      const int cin_r = cin_v;
      for (int ci = 0; ci < cin_r; ++ci) {
        const int xv = (int)sc[xb + ci * hw + p] - xzp;
        for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
          if (j < cn) acc[j] += (int32_t)xv * (int32_t)WT[j][ci];
        }
      }
      for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
        if (j < cn) sc[yb + (co0 + j) * hw + p] = requant(acc[j], MT[j], ST[j], yzp);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 逐通道 3x3
// ---------------------------------------------------------------------------
static void k_dw(const uint8_t* w, uint8_t* sc, int xb, int yb,
                 int C, int stride, int xzp, int yzp,
                 int woff, int boff, int moff, int soff,
                 int hin, int win, int hout, int wout, int padt, int padl) {
#pragma HLS ARRAY_PARTITION variable=WD complete dim=1
#pragma HLS ARRAY_PARTITION variable=BT complete
#pragma HLS ARRAY_PARTITION variable=MT complete
#pragma HLS ARRAY_PARTITION variable=ST complete
  volatile int nblk_v = (C + NCH - 1) / NCH;
  const int nblk = nblk_v;
  for (int b = 0; b < nblk; ++b) {
    const int c0 = b * NCH;
    const int cn = (C - c0 < NCH) ? (C - c0) : NCH;
    for (int j = 0; j < NCH; ++j) {
#pragma HLS UNROLL
      const int ok = (j < cn) ? 1 : 0;
      BT[j] = ok ? bias_at(w, boff + c0 + j) : 0;
      MT[j] = ok ? mult_at(w, moff + c0 + j) : 0;
      ST[j] = ok ? sh_at(w, soff + c0 + j) : (int8_t)0;
      for (int kk = 0; kk < 9; ++kk)
        WD[j][kk / 3][kk % 3] = ok ? (int8_t)w[woff + (c0 + j) * 9 + kk] : (int8_t)0;
    }
    for (int oy = 0; oy < hout; ++oy) {
      for (int ox = 0; ox < wout; ++ox) {
#pragma HLS PIPELINE II=1
        int32_t acc[NCH];
#pragma HLS ARRAY_PARTITION variable=acc complete
        for (int j = 0; j < NCH; ++j) {
#pragma HLS UNROLL
          acc[j] = BT[j];
        }
        for (int ky = 0; ky < 3; ++ky) {
          const int iy = oy * stride - padt + ky;
          if (iy < 0 || iy >= hin) continue;        // 量化域零填充
          for (int kx = 0; kx < 3; ++kx) {
            const int ix = ox * stride - padl + kx;
            if (ix < 0 || ix >= win) continue;
            const int off = iy * win + ix;
            for (int j = 0; j < NCH; ++j) {
#pragma HLS UNROLL
              if (j < cn)
                acc[j] += (int32_t)((int)sc[xb + (c0 + j) * hin * win + off] - xzp) *
                          (int32_t)WD[j][ky][kx];
            }
          }
        }
        for (int j = 0; j < NCH; ++j) {
#pragma HLS UNROLL
          if (j < cn)
            sc[yb + (c0 + j) * hout * wout + oy * wout + ox] =
                requant(acc[j], MT[j], ST[j], yzp);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 首层 3x3 stride2
// ---------------------------------------------------------------------------
static void k_c3(const uint8_t* w, uint8_t* sc, int xb, int yb,
                 int cin, int cout, int stride, int xzp, int yzp,
                 int woff, int boff, int moff, int soff,
                 int hin, int win, int hout, int wout, int padt, int padl) {
#pragma HLS ARRAY_PARTITION variable=BT complete
#pragma HLS ARRAY_PARTITION variable=MT complete
#pragma HLS ARRAY_PARTITION variable=ST complete
  for (int j = 0; j < NLANE; ++j) {
#pragma HLS UNROLL
    const int ok = (j < cout) ? 1 : 0;
    BT[j] = ok ? bias_at(w, boff + j) : 0;
    MT[j] = ok ? mult_at(w, moff + j) : 0;
    ST[j] = ok ? sh_at(w, soff + j) : (int8_t)0;
  }
  for (int oy = 0; oy < hout; ++oy) {
    for (int ox = 0; ox < wout; ++ox) {
#pragma HLS PIPELINE II=1
      int32_t acc[NLANE];
#pragma HLS ARRAY_PARTITION variable=acc complete
      for (int j = 0; j < NLANE; ++j) {
#pragma HLS UNROLL
        acc[j] = BT[j];
      }
      volatile int cin_v = cin;
      const int cin_r = cin_v;
      for (int ci = 0; ci < cin_r; ++ci) {
        for (int ky = 0; ky < 3; ++ky) {
          const int iy = oy * stride - padt + ky;
          if (iy < 0 || iy >= hin) continue;
          for (int kx = 0; kx < 3; ++kx) {
            const int ix = ox * stride - padl + kx;
            if (ix < 0 || ix >= win) continue;
            const int xv = (int)sc[xb + ci * hin * win + iy * win + ix] - xzp;
            for (int j = 0; j < NLANE; ++j) {
#pragma HLS UNROLL
              if (j < cout)
                acc[j] += (int32_t)xv * (int32_t)(int8_t)w[woff + j * cin * 9 + ci * 9 + ky * 3 + kx];
            }
          }
        }
      }
      for (int j = 0; j < NLANE; ++j) {
#pragma HLS UNROLL
        if (j < cout)
          sc[yb + j * hout * wout + oy * wout + ox] = requant(acc[j], MT[j], ST[j], yzp);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 残差 / GAP / FC / 搬运
// ---------------------------------------------------------------------------
static void k_add(uint8_t* sc, int ab, int bb, int yb, int n,
                  int azp, int bzp, int yzp, int am, int bm, int qsh) {
  const int64_t rnd = (int64_t)1 << (qsh - 1);
  for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
    int64_t v = (int64_t)((int)sc[ab + i] - azp) * am + (int64_t)((int)sc[bb + i] - bzp) * bm;
    int64_t r = (v >= 0) ? ((v + rnd) >> qsh) : -(((-v) + rnd) >> qsh);
    int q = (int)(r + yzp);
    if (q < 0) q = 0;
    if (q > 255) q = 255;
    sc[yb + i] = (uint8_t)q;
  }
}

static void k_gap(uint8_t* sc, int xb, int nch, int hw,
                  int xzp, int yzp, int am, int qsh) {
  const int64_t rnd = (int64_t)1 << (qsh - 1);
  for (int i = 0; i < nch; ++i) {
    int32_t acc = 0;
    volatile int hw_v = hw;
    const int hw_r = hw_v;
    for (int j = 0; j < hw_r; ++j)
#pragma HLS PIPELINE II=1
      acc += (int)sc[xb + i * hw + j] - xzp;
    int64_t v = (int64_t)acc * am;
    int64_t r = (v >= 0) ? ((v + rnd) >> qsh) : -(((-v) + rnd) >> qsh);
    int q = (int)(r + yzp);
    if (q < 0) q = 0;
    if (q > 255) q = 255;
    FC_IN[i] = (uint8_t)q;
  }
}

// which: 0 = 72->256 写 FC_HID 再搬回 FC_IN；1 = 256->1 写 gen_FC_P；2 = 256->2 写 gen_FC_H
static void k_fc(const uint8_t* w, int woff, int cin, int cout,
                 int xzp, int yzp, int boff, int moff, int soff, int which) {
  for (int o = 0; o < cout; ++o) {
    int32_t acc = bias_at(w, boff + o);
#pragma HLS PIPELINE II=1
    volatile int cin_v = cin;
    const int cin_r = cin_v;
    for (int i = 0; i < cin_r; ++i) {
      const int xv = (int)FC_IN[i] - xzp;
      acc += (int32_t)xv * (int32_t)(int8_t)w[woff + o * cin + i];
    }
    const uint8_t v = requant(acc, mult_at(w, moff + o), sh_at(w, soff + o), yzp);
    if (which == 0) FC_HID[o] = v;
    else if (which == 1) gen_FC_P[o] = v;
    else gen_FC_H[o] = v;
  }
  // 边读 FC_IN 边写 FC_IN 会把后面的输出算错，所以隐藏层先写 FC_HID 再整体搬回
  if (which == 0) {
    for (int o = 0; o < cout; ++o) {
#pragma HLS PIPELINE II=1
      FC_IN[o] = FC_HID[o];
    }
  }
}

static void k_hm(uint8_t* sc, int xb, int n) {
  for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
    gen_HM_BUF[i] = sc[xb + i];
  }
}

static void k_copy(uint8_t* sc, int srcb, int dstb, int n) {
  for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
    sc[dstb + i] = sc[srcb + i];
  }
}

// 生成的逐层调度（52 行显式调用，全部字面量）
#include "kpt_net_gen.h"

// ---------------------------------------------------------------------------
// 整网入口（不含 HLS 端口，供本地对拍；HLS 顶层在 kpt_hls_gen_top.cpp）
// ---------------------------------------------------------------------------
void forward_gen(const uint8_t* wblob, const uint8_t* img, uint8_t* sc) {
  // 输入是 HWC（相机像素顺序），内部一律 CHW
  for (int y = 0; y < IN_SIZE; ++y)
    for (int x = 0; x < IN_SIZE; ++x)
      for (int ch = 0; ch < 3; ++ch)
        sc[ch * IN_SIZE * IN_SIZE + y * IN_SIZE + x] = img[(y * IN_SIZE + x) * 3 + ch];
  kpt_body(sc, wblob);
}

}  // namespace kptg
