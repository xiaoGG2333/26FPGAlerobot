// 阶段1 手部关键点网 —— IP 功能模型（HLS 之前的 C++ 参考实现）
//
// 这份代码就是将来 HLS 的同一个算法：先和 tools/int8_ref.py 对拍一致，
// 再加 pragma / 换成 line buffer / 定点累加器，才谈综合。
//
// 定点规则（已与 onnxruntime int8 对拍，平均差 0.32 LSB）：
//   激活 uint8 非对称（逐层 zp；ReLU6 输出的那层 zp=0、scale=max/255）
//   权重 int8，zp=0，逐输出通道 scale
//   bias 是 int32（图里已经按 sx*sw 量化过）
//   y = clamp(((acc*mult + round) >> (31+shift)) + y_zp, 0, 255)
//   量化域的 0 填充 = x_zp
#pragma once
#include <stdint.h>
#include <string.h>

#include "kpt_layers.h"
#include "kpt_params.h"

namespace kpt {

enum { IN_SIZE = 192, HM = 12, N_KPT = 21 };

// 从 bin 里切出来的三个视图
struct Params {
  const int8_t* w;     // 权重
  const int32_t* b;    // bias
  const int32_t* mult; // requant 乘法系数
  const int8_t* sh;    // requant 右移
};

static inline void load_params(const uint8_t* blob, Params& p) {
  p.w = reinterpret_cast<const int8_t*>(blob);
  p.b = reinterpret_cast<const int32_t*>(blob + KPT_OFF_BIAS);
  p.mult = reinterpret_cast<const int32_t*>(blob + KPT_OFF_MULT);
  p.sh = reinterpret_cast<const int8_t*>(blob + KPT_OFF_SHIFT);
}

static inline uint8_t requant(int64_t acc, int32_t mult, int8_t sh, int y_zp) {
  const int64_t rnd = (sh > 0) ? ((int64_t)1 << (30 + sh)) : 0;
  int64_t y = (acc * (int64_t)mult + rnd) >> (31 + sh);
  y += y_zp;
  if (y < 0) y = 0;
  if (y > 255) y = 255;
  return (uint8_t)y;
}

// 通用卷积：支持 1x1/3x3/普通/depthwise，stride/pad/group 由层表给出
// x: (cin, hin, win) uint8  →  y: (cout, hout, wout) uint8
void conv_layer(const KptLayer& L, const Params& P, const uint8_t* x, int hin, int win,
                uint8_t* y, int hout, int wout);

void add_layer(const KptLayer& L, const uint8_t* a, const uint8_t* b, uint8_t* y, int n);

void gap_layer(const KptLayer& L, const uint8_t* x, int c, int h, int w, uint8_t* y);

void fc_layer(const KptLayer& L, const Params& P, const uint8_t* x, uint8_t* y);

// 整网：输入 192x192x3 (BGR, 像素值直接就是 uint8 量化值)
// 输出 hm: 21*12*12 uint8 热图 logits；pres: 1；hand: 2
void forward(const KptLayer* layers, int n_layers, const Params& P,
             const uint8_t* img, uint8_t* hm, uint8_t* pres, uint8_t* hand);

// HLS 结构版（kpt_hls.cpp）：同样的层表、同样的定点，只换并行结构
// 注：scratch 是外部 DDR 缓冲（KPT_SCRATCH_BYTES 字节），HLS 版不用 std::vector；
//     wblob 保持 uint8_t*（HLS 不允许在 m_axi 指针上 reinterpret_cast），
//     三路输出经片上静态缓冲由顶层统一写出，所以这里不出现在参数里。
void forward_hls(const KptLayer* layers, int n_layers, const uint8_t* wblob,
                 const uint8_t* img, uint8_t* scratch);

// HLS 版（方案 B，逐层展开直写调度，kpt_hls_gen.cpp）的输出缓冲
extern uint8_t HM_BUF[N_KPT * HM * HM];
extern uint8_t FC_P[1];
extern uint8_t FC_H[2];


}  // namespace kpt

// 方案 B：逐层展开的直写调度（层表不进综合，循环边界全是字面量）
// 输出落在 gen_HM_BUF / gen_FC_P / gen_FC_H（片上静态缓冲）。
// 注意：在 kpt 之外另开一个 namespace，否则和 kpt::HM_BUF 那套撞名字。
namespace kptg {
void forward_gen(const uint8_t* wblob, const uint8_t* img, uint8_t* scratch);
extern uint8_t gen_HM_BUF[21 * 12 * 12];
extern uint8_t gen_FC_P[1];
extern uint8_t gen_FC_H[2];
}  // namespace kptg

// HLS 顶层（hls.tcl 的 set_top kpt_net_entry）：权重/输入/输出都按 AXI 主口拖出去
extern "C" void kpt_net_entry(const uint8_t* wblob, const uint8_t* img, uint8_t* scratch,
                              uint8_t* hm, uint8_t* pres, uint8_t* hand);
