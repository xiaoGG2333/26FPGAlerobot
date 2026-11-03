// 阶段1 手部关键点网 —— 单层基线（只为拿真实资源/时序数字）
//
// 目的：整网 52 层全内联的综合太慢（两个方案都跑了几小时没结果），
//       先用"最有代表性的一层"综合出 DSP48 / BRAM / FF / LUT / II 的真实数字，
//       再按 MAC 比例外推整网，不用盲等。
//
// 两个顶层（分别综合，各自出一个 csynth.rpt）：
//   kpt_base_pw  逐点 1x1  16->96 @96x96   全网最重的一层，14.16M MACs
//   kpt_base_dw  逐通道 3x3 96ch @96x96 stride2 -> 48x48   13 个 DW 层的代表
//
// 与整网 kernel 的唯一区别：读、写分成两个 m_axi 口（src/dst），
// 因为整网是乒乓缓冲，这里单层没法乒乓。算术、并行度、requant 完全一致。
#include "kpt_net.h"

using kpt::requant;

#define NMAC 64
#define NCH  32
#define CIN_MAX 512

static int8_t  WT[NMAC][CIN_MAX];
static int32_t BT[NMAC];
static int32_t MT[NMAC];
static int8_t  ST[NMAC];
static int8_t  WD[NCH][3][3];

static inline int32_t rd32(const uint8_t* p, int off) {
  uint32_t v = (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) |
               ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
  return (int32_t)v;
}
static inline int32_t bias_at(const uint8_t* w, int i) { return rd32(w, KPT_OFF_BIAS + 4 * i); }
static inline int32_t mult_at(const uint8_t* w, int i) { return rd32(w, KPT_OFF_MULT + 4 * i); }
static inline int8_t  sh_at(const uint8_t* w, int i) { return (int8_t)w[KPT_OFF_SHIFT + i]; }

// ---------------------------------------------------------------------------
// 逐点 1x1
// ---------------------------------------------------------------------------
static void k_pw(const uint8_t* w, const uint8_t* src, uint8_t* dst,
                 int xb, int yb, int cin, int cout, int xzp, int yzp,
                 int woff, int boff, int moff, int soff, int hw) {
#pragma HLS ARRAY_PARTITION variable=WT complete dim=1
#pragma HLS ARRAY_PARTITION variable=BT complete
#pragma HLS ARRAY_PARTITION variable=MT complete
#pragma HLS ARRAY_PARTITION variable=ST complete
  volatile int nblk_v = (cout + NMAC - 1) / NMAC;
  const int nblk = nblk_v;
  for (int b = 0; b < nblk; ++b) {
    const int co0 = b * NMAC;
    const int cn = (cout - co0 < NMAC) ? (cout - co0) : NMAC;
    // 权重复制到片上（每个输出通道块一次）
    for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
      const int ok = (j < cn) ? 1 : 0;
      BT[j] = ok ? bias_at(w, boff + co0 + j) : 0;
      MT[j] = ok ? mult_at(w, moff + co0 + j) : 0;
      ST[j] = ok ? sh_at(w, soff + co0 + j) : (int8_t)0;
      volatile int cin_v = cin;
      const int cin_r = cin_v;
      for (int ci = 0; ci < cin_r; ++ci) {
#pragma HLS PIPELINE II=1
        WT[j][ci] = ok ? (int8_t)w[woff + (co0 + j) * cin + ci] : (int8_t)0;
      }
    }
    // 逐像素 1x1
    for (int p = 0; p < hw; ++p) {
#pragma HLS PIPELINE II=1
      int32_t acc[NMAC];
#pragma HLS ARRAY_PARTITION variable=acc complete
      for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
        acc[j] = BT[j];
      }
      volatile int cin_v = cin;
      const int cin_r = cin_v;
      for (int ci = 0; ci < cin_r; ++ci) {
        const int xv = (int)src[xb + ci * hw + p] - xzp;
        for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
          if (j < cn) acc[j] += (int32_t)xv * (int32_t)WT[j][ci];
        }
      }
      for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
        if (j < cn) dst[yb + (co0 + j) * hw + p] = requant(acc[j], MT[j], ST[j], yzp);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 逐通道 3x3
// ---------------------------------------------------------------------------
static void k_dw(const uint8_t* w, const uint8_t* src, uint8_t* dst,
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
          if (iy < 0 || iy >= hin) continue;
          for (int kx = 0; kx < 3; ++kx) {
            const int ix = ox * stride - padl + kx;
            if (ix < 0 || ix >= win) continue;
            const int off = iy * win + ix;
            for (int j = 0; j < NCH; ++j) {
#pragma HLS UNROLL
              if (j < cn)
                acc[j] += (int32_t)((int)src[(c0 + j) * hin * win + off] - xzp) *
                          (int32_t)WD[j][ky][kx];
            }
          }
        }
        for (int j = 0; j < NCH; ++j) {
#pragma HLS UNROLL
          if (j < cn)
            dst[(c0 + j) * hout * wout + oy * wout + ox] =
                requant(acc[j], MT[j], ST[j], yzp);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 顶层 1：逐点 1x1  16 -> 96 @ 96x96
// ---------------------------------------------------------------------------
extern "C" void kpt_base_pw(const uint8_t* w, const uint8_t* src, uint8_t* dst) {
#pragma HLS INTERFACE m_axi port=w   offset=slave bundle=gmem_w   depth=1536
#pragma HLS INTERFACE m_axi port=src offset=slave bundle=gmem_in  depth=147456
#pragma HLS INTERFACE m_axi port=dst offset=slave bundle=gmem_out depth=884736
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
#pragma HLS INLINE recursive
  k_pw(w, src, dst, 0, 0, 16, 96, 131, 0, 0, 0, 0, 0, 9216);
}

// ---------------------------------------------------------------------------
// 顶层 2：逐通道 3x3  96ch @ 96x96 stride2 -> 48x48
// ---------------------------------------------------------------------------
extern "C" void kpt_base_dw(const uint8_t* w, const uint8_t* src, uint8_t* dst) {
#pragma HLS INTERFACE m_axi port=w   offset=slave bundle=gmem_w   depth=864
#pragma HLS INTERFACE m_axi port=src offset=slave bundle=gmem_in  depth=884736
#pragma HLS INTERFACE m_axi port=dst offset=slave bundle=gmem_out depth=221184
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
#pragma HLS INLINE recursive
  k_dw(w, src, dst, 96, 2, 0, 0, 0, 0, 0, 0, 96, 96, 48, 48, 1, 1);
}
