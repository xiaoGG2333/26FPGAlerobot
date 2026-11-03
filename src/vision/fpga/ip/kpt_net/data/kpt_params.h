// 自动生成，勿手改（tools/gen_ip_header.py）
// 源模型：results/stage1/self_run5_192/int8/best_hm_int8.onnx
#pragma once
#include <stdint.h>

static const int KPT_WEIGHT_BYTES = 331224;
static const int KPT_BIAS_WORDS   = 6536;
static const int KPT_BLOB_BYTES   = 390048;
static const int KPT_OFF_BIAS     = 331224;
static const int KPT_OFF_MULT     = 357368;
static const int KPT_OFF_SHIFT    = 383512;
static const int KPT_LAYER_COUNT  = 52;

// 4 个独立权重口：HLS 不允许在 m_axi 指针上做 reinterpret_cast，
// 所以 int8 权重 / int32 bias / int32 mult / int8 shift 各走一个口
static const int KPT_W_BYTES    = 331224;
static const int KPT_B_WORDS    = 6536;
static const int KPT_M_WORDS    = 6536;
static const int KPT_S_BYTES    = 6536;
static const int KPT_BLOB_WORDS = 97512;

// 激活缓冲（外部 DDR，V2.5 起不再用 std::vector）
static const int KPT_MAX_ACT   = 884736;
static const int KPT_MAX_SKIP  = 147456;
static const int KPT_SCRATCH_A    = 0;
static const int KPT_SCRATCH_B    = KPT_MAX_ACT;
static const int KPT_SCRATCH_SKIP = KPT_MAX_ACT * 2;
static const int KPT_SCRATCH_BYTES = KPT_MAX_ACT * 2 + KPT_MAX_SKIP;

