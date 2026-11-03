// V2 对拍台：HLS 结构版 vs 功能模型，必须【逐位一致】（差 0）
//
// 用法：tb_kpt_hls <weights.bin> <input.bin>
//   input.bin = 192*192*3 uint8（HWC，BGR）
// 输出：逐元素比较 hm/pres/hand，打印最大/平均差，退出码 0 = 通过
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "kpt_net.h"

static std::vector<uint8_t> read_file(const char* path, size_t n) {
  std::vector<uint8_t> v(n);
  FILE* f = fopen(path, "rb");
  if (!f) { printf("打不开 %s\n", path); exit(2); }
  size_t got = fread(v.data(), 1, n, f);
  fclose(f);
  if (got != n) { printf("%s 只有 %zu 字节，期望 %zu\n", path, got, n); exit(2); }
  return v;
}

int main(int argc, char** argv) {
  if (argc < 3) { printf("用法：%s <weights.bin> <input.bin>\n", argv[0]); return 2; }
  std::vector<uint8_t> blob = read_file(argv[1], KPT_BLOB_BYTES);
  std::vector<uint8_t> img = read_file(argv[2], (size_t)kpt::IN_SIZE * kpt::IN_SIZE * 3);

  kpt::Params P;
  kpt::load_params(blob.data(), P);
  const int NHM = kpt::N_KPT * kpt::HM * kpt::HM;

  std::vector<uint8_t> hm_a(NHM), hm_b(NHM), hm_c(NHM);
  std::vector<uint8_t> scratch(KPT_SCRATCH_BYTES);   // 模拟 PS 分配的 DDR 缓冲
  uint8_t pa = 0, pb = 0, pc = 0, ha[2] = {0, 0}, hb[2] = {0, 0}, hc[2] = {0, 0};

  kpt::forward(KPT_LAYERS, KPT_LAYER_COUNT, P, img.data(), hm_a.data(), &pa, ha);

  // V2：层表驱动版
  kpt::forward_hls(KPT_LAYERS, KPT_LAYER_COUNT, blob.data(), img.data(), scratch.data());
  for (int i = 0; i < NHM; ++i) hm_b[i] = kpt::HM_BUF[i];   // 输出落在片上缓冲
  pb = kpt::FC_P[0];
  hb[0] = kpt::FC_H[0];
  hb[1] = kpt::FC_H[1];

  // V3：逐层展开直写调度版
  kptg::forward_gen(blob.data(), img.data(), scratch.data());
  for (int i = 0; i < NHM; ++i) hm_c[i] = kptg::gen_HM_BUF[i];
  pc = kptg::gen_FC_P[0];
  hc[0] = kptg::gen_FC_H[0];
  hc[1] = kptg::gen_FC_H[1];

  long long sum = 0;
  int mx = 0, n_gt0 = 0;
  for (int i = 0; i < NHM; ++i) {
    int d = (int)hm_a[i] - (int)hm_b[i];
    if (d < 0) d = -d;
    sum += d;
    if (d > mx) mx = d;
    if (d > 0) ++n_gt0;
  }
  printf("热图: 最大差 %d LSB，平均 %.4f LSB，不一致元素 %d/%d (%.2f%%)\n",
         mx, (double)sum / NHM, n_gt0, NHM, 100.0 * n_gt0 / NHM);
  printf("presence: 功能模型 %d  结构版 %d\n", (int)pa, (int)pb);
  printf("handed  : 功能模型 [%d,%d]  结构版 [%d,%d]\n", (int)ha[0], (int)ha[1],
         (int)hb[0], (int)hb[1]);
  const bool ok = (mx == 0) && (pa == pb) && (ha[0] == hb[0]) && (ha[1] == hb[1]);
  printf("[结论] V2 %s\n", ok ? "通过：层表版与功能模型逐位一致" : "未通过：还有不一致");

  // ---- V3：逐层展开版 ----
  long long sum3 = 0;
  int mx3 = 0, n3 = 0;
  for (int i = 0; i < NHM; ++i) {
    int d = (int)hm_a[i] - (int)hm_c[i];
    if (d < 0) d = -d;
    sum3 += d;
    if (d > mx3) mx3 = d;
    if (d > 0) ++n3;
  }
  printf("gen热图: 最大差 %d LSB，平均 %.4f LSB，不一致元素 %d/%d (%.2f%%)\n",
         mx3, (double)sum3 / NHM, n3, NHM, 100.0 * n3 / NHM);
  printf("genpres: 功能模型 %d  展开版 %d\n", (int)pa, (int)pc);
  printf("genhand: 功能模型 [%d,%d]  展开版 [%d,%d]\n", (int)ha[0], (int)ha[1],
         (int)hc[0], (int)hc[1]);
  const bool ok3 = (mx3 == 0) && (pa == pc) && (ha[0] == hc[0]) && (ha[1] == hc[1]);
  printf("[结论] V3 %s\n", ok3 ? "通过：展开版与功能模型逐位一致" : "未通过：还有不一致");
  return (ok && ok3) ? 0 : 1;
}
