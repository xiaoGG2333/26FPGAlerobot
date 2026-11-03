// 阶段1 手部关键点网 —— 方案 B 的 HLS 顶层
//
// 单独一个文件，只为了让 hls_gen.tcl 能把"顶层 + 端口 pragma"和
// kpt_hls_gen.cpp 一起综合，同时又不和 kpt_hls.cpp 的 kpt_net_entry 撞名
// （两个文件不会同时进同一个工程/同一个可执行程序）。
#include "kpt_net.h"

// ---------------------------------------------------------------------------
// HLS 顶层（hls_gen.tcl 的 set_top kpt_net_entry）
//   wblob = 权重 blob（390048 B）的 DDR 地址；scratch 由 PS 分配 1.83 MB；
//   hm/pres/hand 由 PS 读走。
// ---------------------------------------------------------------------------
extern "C" void kpt_net_entry(const uint8_t* wblob, const uint8_t* img, uint8_t* scratch,
                              uint8_t* hm, uint8_t* pres, uint8_t* hand) {
#pragma HLS INTERFACE m_axi port=wblob   offset=slave bundle=gmem_w   depth=KPT_BLOB_BYTES
#pragma HLS INTERFACE m_axi port=img     offset=slave bundle=gmem_in  depth=110592
#pragma HLS INTERFACE m_axi port=scratch offset=slave bundle=gmem_act depth=KPT_SCRATCH_BYTES
#pragma HLS INTERFACE m_axi port=hm      offset=slave bundle=gmem_out depth=3024
#pragma HLS INTERFACE m_axi port=pres    offset=slave bundle=gmem_out depth=3024
#pragma HLS INTERFACE m_axi port=hand    offset=slave bundle=gmem_out depth=3024
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
// 顶层指针绑成 m_axi 后不能传给"非内联"子函数（HLS 会静默删掉整个调用）
#pragma HLS INLINE recursive

  kptg::forward_gen(wblob, img, scratch);

  for (int i = 0; i < kpt::N_KPT * kpt::HM * kpt::HM; ++i) {
#pragma HLS PIPELINE II=1
    hm[i] = kptg::gen_HM_BUF[i];
  }
  pres[0] = kptg::gen_FC_P[0];
  hand[0] = kptg::gen_FC_H[0];
  hand[1] = kptg::gen_FC_H[1];
}
