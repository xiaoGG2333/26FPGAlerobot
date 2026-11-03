// 功能模型自测台：读 input.bin(192*192*3 像素)，跑整网，写 hm/pres/hand
// 编译（在 fpga/ip/kpt_net 下）：
//   g++ -O2 -std=c++17 -I data -I src tb/tb_kpt_net.cpp src/kpt_net.cpp -o build/tb_kpt_net
#include <cstdio>
#include <vector>

#include "kpt_net.h"

int main(int argc, char** argv) {
  const char* blob_path = (argc > 1) ? argv[1] : "data/kpt_weights.bin";
  const char* in_path = (argc > 2) ? argv[2] : "input.bin";
  const char* out_path = (argc > 3) ? argv[3] : "out.bin";

  FILE* f = fopen(blob_path, "rb");
  if (!f) { printf("打不开 %s\n", blob_path); return 1; }
  std::vector<uint8_t> blob(KPT_BLOB_BYTES);
  if (fread(blob.data(), 1, blob.size(), f) != blob.size()) { printf("权重长度不符\n"); return 1; }
  fclose(f);

  kpt::Params P;
  kpt::load_params(blob.data(), P);

  std::vector<uint8_t> img(3 * kpt::IN_SIZE * kpt::IN_SIZE);
  f = fopen(in_path, "rb");
  if (!f) { printf("打不开 %s\n", in_path); return 1; }
  if (fread(img.data(), 1, img.size(), f) != img.size()) { printf("输入长度不符\n"); return 1; }
  fclose(f);

  std::vector<uint8_t> hm(kpt::N_KPT * kpt::HM * kpt::HM, 0), pres(1, 0), hand(2, 0);
  kpt::forward(KPT_LAYERS, KPT_LAYER_COUNT, P, img.data(), hm.data(),
               pres.data(), hand.data());

  f = fopen(out_path, "wb");
  fwrite(hm.data(), 1, hm.size(), f);
  fwrite(pres.data(), 1, 1, f);
  fwrite(hand.data(), 1, 2, f);
  fclose(f);
  printf("[完成] hm %d B -> %s   pres=%d hand=%d,%d\n", (int)hm.size(), out_path,
         pres[0], hand[0], hand[1]);
  return 0;
}
