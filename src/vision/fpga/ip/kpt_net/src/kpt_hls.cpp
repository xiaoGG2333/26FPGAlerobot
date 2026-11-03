// 阶段1 手部关键点网 —— HLS 结构版实现（V2.6）
//
// 与 kpt_net.cpp（功能模型）的区别只在"怎么算"，不在"算什么"：
//   - 逐点层按【输出通道 64 路并行】算（NMAC=64），权重块先搬进片上 BRAM
//   - 逐通道 3x3 层按【通道 32 路并行】算（NCH=32）
//   - 首层 3x3 stride2 按【输出通道 24 路并行】算
//
// 本文件必须与功能模型【逐位一致】：tb/tb_kpt_hls.cpp 把两者链接到一起直接对拍。
//
// ==== V2.6 为什么改（前三次综合失败的原因）=================================
// 1) 顶层指针一旦绑成 m_axi，就不能再传给"非内联"的子函数：HLS 建不起连接，
//    会把整个调用当无副作用剪掉（SYNCHK 200-77 no outputs、Latency 0、
//    四个 gmem bundle 全悬空、RTL 里没有任何乘法器）。→ 顶层加 INLINE recursive。
// 2) 内联后暴露真问题：`reinterpret_cast<const int32_t*>(blob + off)` 被判
//    SYNCHK 200-41 不支持。→ 权重口一律保持 uint8_t*，int32 字段用 rd32() 逐字节拼。
// 3) 更关键：`cur`/`nxt`/`skip` 这种"指针变量乒乓"会被判 SYNCHK 200-43
//    "non-static pointer" + 200-61 "unknown size"。→ 全部改成【整数偏移】，
//    任何访存都写成 sc[base + i] 的形式，base 是 int。
//
// 缓冲区：激活张量放 DDR（PS DDR 经 AXI HP），片上只留权重块、bias/mult/shift 块
//   和 3 行窗口。每帧激活读写 ~15-20 MB（30fps 约 500 MB/s），DDR 够；缺的是 DSP。
//   三路输出先落在片上静态数组（HM_BUF/FC_P/FC_H），由顶层统一写端口，
//   这样"谁写了顶层输出"对 SYNCHK 一目了然。
//
// g++ 不知道怎么处理 #pragma HLS，编译时加 -Wno-unknown-pragmas 即可。
#include <cstdio>

#include "kpt_net.h"


namespace kpt {

// ---- 并行度（HLS 综合参数）-------------------------------------------------
#ifndef NMAC
#define NMAC 64          // 逐点层：输出通道并行度
#endif
#ifndef NCH
#define NCH 32           // 逐通道层：通道并行度
#endif
#define NLANE 32         // 首层 conv3x3 s2：输出通道并行度（cout=24）
#define CIN_MAX 512      // 最大输入通道数（本网 432）

// 片上权重/参数块（综合后是 BRAM；C-sim 里就是普通内存）
static int8_t  WT[NMAC][CIN_MAX];
static int32_t BT[NMAC];      // bias
static int32_t MT[NMAC];      // 乘子
static int8_t  ST[NMAC];      // 移位
static int8_t  WD[NCH][3][3];

// 片上输出小缓冲
static uint8_t FC_IN[256];              // 片上：pres/hand 两个头共用的 256 维输入
static uint8_t FC_HID[256];             // 72->256 隐藏层的输出（不能直接写回 FC_IN，会互相踩）
// 下面三个不加 static：tb 要把 HLS 版的输出取出来和功能模型逐位对拍
uint8_t HM_BUF[N_KPT * HM * HM];        // 21x12x12 热图
uint8_t FC_P[1];
uint8_t FC_H[2];

// 在 uint8 口上读一个小端 int32（HLS 不允许在 m_axi 指针上 reinterpret_cast）
static inline int32_t rd32(const uint8_t* p, int off) {
  uint32_t v = (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) |
               ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
  return (int32_t)v;
}

// bias/mult/shift 三家的 off 都是【各自段内的索引】（功能模型是 P.b + L.b_off 这种），
// 不是 blob 里的绝对字节偏移，所以这里必须加段基址。
static inline int32_t bias_at(const KptLayer& L, const uint8_t* wblob, int i) {
  return rd32(wblob, KPT_OFF_BIAS + 4 * (L.b_off + i));
}
static inline int32_t mult_at(const KptLayer& L, const uint8_t* wblob, int i) {
  return rd32(wblob, KPT_OFF_MULT + 4 * (L.m_off + i));
}
static inline int8_t sh_at(const KptLayer& L, const uint8_t* wblob, int i) {
  return (int8_t)wblob[KPT_OFF_SHIFT + L.s_off + i];
}

// ---------------------------------------------------------------------------
// 逐点 1x1：y[co][p] = requant(Σ_ci (x[ci][p]-x_zp)*w[co][ci] + b[co])
// ---------------------------------------------------------------------------
static void hls_pw_layer(const KptLayer& L, uint8_t* sc, int xb, int yb,
                         const uint8_t* wblob, int hw) {
  // 读在 xb 区、写在 yb 区（A/B 乒乓，物理上不重叠），但 xb/yb 是运行时才交换的
  // 变量，HLS 证明不了不重叠 -> 保守判成迭代间依赖，p 循环 II 被压到 256。
  // 这里显式声明：迭代之间无依赖。
#pragma HLS DEPENDENCE variable=sc inter false
#pragma HLS ARRAY_PARTITION variable=WT complete dim=1
#pragma HLS ARRAY_PARTITION variable=BT complete
#pragma HLS ARRAY_PARTITION variable=MT complete
#pragma HLS ARRAY_PARTITION variable=ST complete
  const int cin = L.cin, cout = L.cout;
  const int xzp = L.x_zp;

  for (int co0 = 0; co0 < cout; co0 += NMAC) {
    const int cn = (cout - co0 < NMAC) ? (cout - co0) : NMAC;
    // 权重/参数块搬进 BRAM：每层每块只搬一次，与像素数无关
    for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
      const int ok = (j < cn) ? 1 : 0;
      BT[j] = ok ? bias_at(L, wblob, co0 + j) : 0;
      MT[j] = ok ? mult_at(L, wblob, co0 + j) : 0;
      ST[j] = ok ? sh_at(L, wblob, co0 + j) : (int8_t)0;
      for (int ci = 0; ci < cin; ++ci) {
#pragma HLS PIPELINE II=1
        WT[j][ci] = ok ? (int8_t)wblob[L.w_off + (co0 + j) * cin + ci] : (int8_t)0;
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
      for (int ci = 0; ci < cin; ++ci) {
        const int xv = (int)sc[xb + ci * hw + p] - xzp;
        for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
          if (j < cn) acc[j] += (int32_t)xv * (int32_t)WT[j][ci];
        }
      }
      for (int j = 0; j < NMAC; ++j) {
#pragma HLS UNROLL
        if (j < cn) sc[yb + (co0 + j) * hw + p] = requant(acc[j], MT[j], ST[j], L.y_zp);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 逐通道 3x3（group == cin）：每输出行读 3 行输入窗口，通道 32 路并行
// ---------------------------------------------------------------------------
static void hls_dw_layer(const KptLayer& L, uint8_t* sc, int xb, int yb,
                         const uint8_t* wblob, int hin, int win, int hout, int wout) {
  // 同 hls_pw_layer：读写分处 A/B 两个乒乓区，运行时才确定，HLS 只能保守判依赖。
  // 不声明的话 ox 循环 II=1 会被压到 256（SCHED 204-68）。
#pragma HLS DEPENDENCE variable=sc inter false
#pragma HLS ARRAY_PARTITION variable=WD complete dim=1
#pragma HLS ARRAY_PARTITION variable=BT complete
#pragma HLS ARRAY_PARTITION variable=MT complete
#pragma HLS ARRAY_PARTITION variable=ST complete
  const int C = L.cout;   // 逐通道层 cin == cout
  const int s = L.stride;
  const int xzp = L.x_zp;

  for (int c0 = 0; c0 < C; c0 += NCH) {
    const int cn = (C - c0 < NCH) ? (C - c0) : NCH;
    for (int j = 0; j < NCH; ++j) {
#pragma HLS UNROLL
      const int ok = (j < cn) ? 1 : 0;
      BT[j] = ok ? bias_at(L, wblob, c0 + j) : 0;
      MT[j] = ok ? mult_at(L, wblob, c0 + j) : 0;
      ST[j] = ok ? sh_at(L, wblob, c0 + j) : (int8_t)0;
      for (int kk = 0; kk < 9; ++kk)
        WD[j][kk / 3][kk % 3] = ok ? (int8_t)wblob[L.w_off + (c0 + j) * 9 + kk] : (int8_t)0;
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
          const int iy = oy * s - L.pad_t + ky;
          if (iy < 0 || iy >= hin) continue;          // 量化域零填充：本 tap 贡献为 0
          for (int kx = 0; kx < 3; ++kx) {
            const int ix = ox * s - L.pad_l + kx;
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
                requant(acc[j], MT[j], ST[j], L.y_zp);
        }
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 首层 3x3 stride2（cin=3，cout=24）：24 路输出通道并行
// ---------------------------------------------------------------------------
static void hls_conv3x3s2_layer(const KptLayer& L, uint8_t* sc, int xb, int yb,
                                const uint8_t* wblob, int hin, int win,
                                int hout, int wout) {
  // 同上：A/B 乒乓区读写，声明无迭代间依赖。
#pragma HLS DEPENDENCE variable=sc inter false
#pragma HLS ARRAY_PARTITION variable=BT complete
#pragma HLS ARRAY_PARTITION variable=MT complete
#pragma HLS ARRAY_PARTITION variable=ST complete
  const int cin = L.cin, cout = L.cout;
  const int s = L.stride;
  const int xzp = L.x_zp;

  for (int j = 0; j < NLANE; ++j) {
#pragma HLS UNROLL
    const int ok = (j < cout) ? 1 : 0;
    BT[j] = ok ? bias_at(L, wblob, j) : 0;
    MT[j] = ok ? mult_at(L, wblob, j) : 0;
    ST[j] = ok ? sh_at(L, wblob, j) : (int8_t)0;
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
      for (int ci = 0; ci < cin; ++ci) {
        for (int ky = 0; ky < 3; ++ky) {
          const int iy = oy * s - L.pad_t + ky;
          if (iy < 0 || iy >= hin) continue;
          for (int kx = 0; kx < 3; ++kx) {
            const int ix = ox * s - L.pad_l + kx;
            if (ix < 0 || ix >= win) continue;
            const int xv = (int)sc[xb + ci * hin * win + iy * win + ix] - xzp;
            for (int j = 0; j < NLANE; ++j) {
#pragma HLS UNROLL
              if (j < cout)
                acc[j] += (int32_t)xv *
                          (int32_t)(int8_t)wblob[L.w_off + j * cin * 9 + ci * 9 + ky * 3 + kx];
            }
          }
        }
      }
      for (int j = 0; j < NLANE; ++j) {
#pragma HLS UNROLL
        if (j < cout)
          sc[yb + j * hout * wout + oy * wout + ox] = requant(acc[j], MT[j], ST[j], L.y_zp);
      }
    }
  }
}

// ---------------------------------------------------------------------------
// 残差 / GAP / FC（计算量小，结构照抄功能模型，只加流水 pragma）
// ---------------------------------------------------------------------------
static void hls_add_layer(const KptLayer& L, uint8_t* sc, int ab, int bb, int yb, int n) {
  // 读 ab(SKIP)/bb(A) 区，写 yb(B) 区，三者物理不重叠。
#pragma HLS DEPENDENCE variable=sc inter false
  const int S = L.qsh;
  const int64_t rnd = (int64_t)1 << (S - 1);
  for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
    int64_t v = (int64_t)((int)sc[ab + i] - L.a_zp) * L.am +
                (int64_t)((int)sc[bb + i] - L.b_zp) * L.bm;
    int64_t r = (v >= 0) ? ((v + rnd) >> S) : -(((-v) + rnd) >> S);
    int q = (int)(r + L.y_zp);
    if (q < 0) q = 0;
    if (q > 255) q = 255;
    sc[yb + i] = (uint8_t)q;
  }
}

static void hls_gap_layer(const KptLayer& L, uint8_t* sc, int xb, int c, int hw) {
  const int S = L.qsh;
  const int64_t rnd = (int64_t)1 << (S - 1);
  for (int i = 0; i < c; ++i) {
    int32_t acc = 0;
    for (int j = 0; j < hw; ++j)
#pragma HLS PIPELINE II=1
      acc += (int)sc[xb + i * hw + j] - L.x_zp;
    int64_t v = (int64_t)acc * L.am;
    int64_t r = (v >= 0) ? ((v + rnd) >> S) : -(((-v) + rnd) >> S);
    int q = (int)(r + L.y_zp);
    if (q < 0) q = 0;
    if (q > 255) q = 255;
    FC_IN[i] = (uint8_t)q;
  }
}

// which: 0 = 72->256 写 FC_IN；1 = 256->1 写 FC_P；2 = 256->2 写 FC_H
// 三个 FC 的输入一律取 GAP 写下的 FC_IN（原先 which==0 误从 sc[xb..] 取，是 bug）
static void hls_fc_layer(const KptLayer& L, int which, const uint8_t* wblob) {
  const int cin = L.cin, cout = L.cout;
  for (int o = 0; o < cout; ++o) {
    int32_t acc = bias_at(L, wblob, o);
#pragma HLS PIPELINE II=1
    for (int i = 0; i < cin; ++i) {
      const int xv = (int)FC_IN[i] - L.x_zp;
      acc += (int32_t)xv * (int32_t)(int8_t)wblob[L.w_off + o * cin + i];
    }
    const uint8_t v = requant(acc, mult_at(L, wblob, o), sh_at(L, wblob, o), L.y_zp);
    if (which == 0) FC_HID[o] = v;
    else if (which == 1) FC_P[o] = v;
    else FC_H[o] = v;
  }
  // 隐藏层算完再整体搬回 FC_IN：边读 FC_IN 边写 FC_IN 会把后面的输出算错
  if (which == 0) {
    for (int o = 0; o < cout; ++o) {
#pragma HLS PIPELINE II=1
      FC_IN[o] = FC_HID[o];
    }
  }
}

static void copy_bytes(uint8_t* sc, int srcb, int dstb, int n) {
  // srcb/dstb 是不同的乒乓区（cb -> sb），物理不重叠。
#pragma HLS DEPENDENCE variable=sc inter false
  for (int i = 0; i < n; ++i) {
#pragma HLS PIPELINE II=1
    sc[dstb + i] = sc[srcb + i];
  }
}

// 逐层跟踪（本地调试用：-DKPT_HLS_TRACE 时打印，与功能模型的 KPT_TRACE 对拍）
static void hls_trace(int li, int kind, const uint8_t* p, int n, int c, int h, int w) {
#ifdef KPT_HLS_TRACE
  int mn = 255, mx = 0;
  long long sum = 0;
  for (int i = 0; i < n; ++i) {
    int v = p[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
  }
  printf("  L%02d k%d %dx%dx%d  [%d,%d] mean %.1f\n", li, kind, c, h, w, mn, mx,
         n ? (double)sum / n : 0.0);
#else
  (void)li; (void)kind; (void)p; (void)n; (void)c; (void)h; (void)w;
#endif
}

// ---------------------------------------------------------------------------
// 调度：与功能模型逐层对齐（skip/hm/GAP 顺序完全一致）
//   激活全在外部 DDR（scratch，A/B 乒乓 + SKIP 存跳跃支路），用【整数偏移】表示，
//   不使用任何指向内部的指针变量（HLS 会把它们判为 unknown size）。
// ---------------------------------------------------------------------------
void forward_hls(const KptLayer* layers, int n_layers, const uint8_t* wblob,
                 const uint8_t* img, uint8_t* sc) {
  int cb = KPT_SCRATCH_A;
  int nb = KPT_SCRATCH_B;
  const int sb = KPT_SCRATCH_SKIP;

  // 输入是 HWC（相机像素顺序），内部一律 CHW
  for (int y = 0; y < IN_SIZE; ++y)
    for (int x = 0; x < IN_SIZE; ++x)
      for (int ch = 0; ch < 3; ++ch)
        sc[cb + ch * IN_SIZE * IN_SIZE + y * IN_SIZE + x] = img[(y * IN_SIZE + x) * 3 + ch];

  int c = 3, h = IN_SIZE, w = IN_SIZE;

  for (int li = 0; li < n_layers; ++li) {
    const KptLayer& L = layers[li];
    if (L.kind == 0 || L.kind == 1 || L.kind == 2) {
      if (L.kind == 1 && L.cout > L.cin) copy_bytes(sc, cb, sb, c * h * w);  // 开残差块
      const int k = (L.kind == 1) ? 1 : 3;
      const int s = L.stride;
      const int hout = (h + L.pad_t + L.pad_b - k) / s + 1;
      const int wout = (w + L.pad_l + L.pad_r - k) / s + 1;
      if (L.kind == 2) {
        hls_dw_layer(L, sc, cb, nb, wblob, h, w, hout, wout);
      } else if (L.kind == 1) {
        hls_pw_layer(L, sc, cb, nb, wblob, h * w);
      } else {
        hls_conv3x3s2_layer(L, sc, cb, nb, wblob, h, w, hout, wout);
      }
      if (L.cout == N_KPT) {                    // 热图支路，不吃掉主干
        for (int i = 0; i < L.cout * hout * wout; ++i)
#pragma HLS PIPELINE II=1
          HM_BUF[i] = sc[nb + i];
        hls_trace(li, L.kind, HM_BUF, L.cout * hout * wout, L.cout, hout, wout);
      } else {
        const int t = cb; cb = nb; nb = t;      // 乒乓
        c = L.cout; h = hout; w = wout;
        hls_trace(li, L.kind, sc + cb, c * h * w, c, h, w);
      }
    } else if (L.kind == 3) {                   // 残差
      hls_add_layer(L, sc, sb, cb, nb, c * h * w);
      const int t = cb; cb = nb; nb = t;
      hls_trace(li, L.kind, sc + cb, c * h * w, c, h, w);
    } else if (L.kind == 4) {                   // GAP
      hls_gap_layer(L, sc, cb, c, h * w);
      hls_trace(li, L.kind, FC_IN, c, c, 1, 1);
      c = 1; h = 1; w = 1;
    } else {                                    // FC：1 个隐藏层 + pres/hand 并列
      if (L.cout == 256) {
        hls_fc_layer(L, 0, wblob);
        hls_trace(li, L.kind, FC_IN, L.cout, L.cout, 1, 1);
      } else if (L.cout == 1) {
        hls_fc_layer(L, 1, wblob);
        hls_trace(li, L.kind, FC_P, 1, 1, 1, 1);
      } else {
        hls_fc_layer(L, 2, wblob);
        hls_trace(li, L.kind, FC_H, 2, 2, 1, 1);
      }
    }
  }
}

}  // namespace kpt

// ---------------------------------------------------------------------------
// HLS 顶层（hls.tcl 的 set_top kpt_net_entry）
//   pragma 必须挂在顶层函数上；本地 g++ 编译时 pragma 被忽略，指针就是普通内存。
//   上板时 wblob=权重 blob(390048 B) 的 DDR 地址，scratch 由 PS 分配 1.83 MB
//   （KPT_SCRATCH_BYTES = 2*884736 + 147456），hm/pres/hand 由 PS 读走。
// ---------------------------------------------------------------------------
extern "C" void kpt_net_entry(const uint8_t* wblob, const uint8_t* img, uint8_t* scratch,
                              uint8_t* hm, uint8_t* pres, uint8_t* hand) {
// m_axi 端口不要再叠 s_axilite：offset=slave 已经会给每个口生成 AXI-Lite 地址寄存器，
// 日志里那句 "hm to s_axilite & ap_none" 描述的就是这个寄存器，是正常的。
#pragma HLS INTERFACE m_axi port=wblob   offset=slave bundle=gmem_w   depth=KPT_BLOB_BYTES
#pragma HLS INTERFACE m_axi port=img     offset=slave bundle=gmem_in  depth=110592
#pragma HLS INTERFACE m_axi port=scratch offset=slave bundle=gmem_act depth=KPT_SCRATCH_BYTES
// hm/pres/hand 共用一个输出口；depth 统一写 3024，否则 2018.3 会报 200-466 深度不一致
#pragma HLS INTERFACE m_axi port=hm      offset=slave bundle=gmem_out depth=3024
#pragma HLS INTERFACE m_axi port=pres    offset=slave bundle=gmem_out depth=3024
#pragma HLS INTERFACE m_axi port=hand    offset=slave bundle=gmem_out depth=3024
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
// 关键：顶层指针已绑成 m_axi 后，就不能再传给"非内联"的子函数（HLS 建不起连接，
// 会静默把整个调用剪掉）。递归内联让子函数边界消失，所有访存都落回顶层端口。
#pragma HLS INLINE recursive

  kpt::forward_hls(KPT_LAYERS, KPT_LAYER_COUNT, wblob, img, scratch);

  // 三路输出集中在顶层写，SYNCHK 一眼能看到顶层有输出
  for (int i = 0; i < kpt::N_KPT * kpt::HM * kpt::HM; ++i) {
#pragma HLS PIPELINE II=1
    hm[i] = kpt::HM_BUF[i];
  }
  pres[0] = kpt::FC_P[0];
  hand[0] = kpt::FC_H[0];
  hand[1] = kpt::FC_H[1];
}
