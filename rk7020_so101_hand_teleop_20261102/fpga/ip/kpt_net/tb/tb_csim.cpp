// HLS C 仿真用 testbench：不靠命令行参数，直接用相对路径（在 fpga/ip/kpt_net 下跑）
//
// 用途：验证 Vivado HLS 的前端（ApCC / gcc 4.6.3）能不能正确读到
//   (1) 自动生成的层表 KPT_LAYERS（52 个结构体的静态常量数组，含 float 成员）
//   (2) 权重 blob 与输入
// 并跑完整网。如果这里 hm 全是 0 或者层表打印不对，说明综合出空电路是**前端问题**，
// 不是我的 pragma 写错。
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "kpt_net.h"

static uint8_t g_blob[KPT_BLOB_BYTES];
static uint8_t g_img[192 * 192 * 3];
static uint8_t g_scratch[KPT_SCRATCH_BYTES];
static uint8_t g_hm[21 * 12 * 12];
static uint8_t g_pres[1];
static uint8_t g_hand[2];

// csim 的工作目录在 kpt_net_csim/solution1/csim/build（和综合无关），
// 所以这里按候选前缀挨个试，而不是写死相对路径。
static const char* kPrefix[] = {"", "../../../../", "../../../../../", "../../../../../.."};

static int load(const char* path, uint8_t* dst, int n) {
  FILE* f = NULL;
  char full[512];
  full[0] = 0;
  for (int i = 0; i < 4 && !f; ++i) {
    sprintf(full, "%s%s", kPrefix[i], path);
    f = fopen(full, "rb");
  }
  if (!f) { printf("打不开 %s\n", path); return 0; }
  int got = (int)fread(dst, 1, n, f);
  fclose(f);
  printf("  读 %-34s %d 字节\n", full, got);
  return got == n;
}

int main() {
  printf("=== 层表自检（ApCC 能不能读到静态常量结构体数组）===\n");
  const int probe[5] = {0, 1, 4, 46, 51};
  for (int i = 0; i < 5; ++i) {
    const KptLayer& L = KPT_LAYERS[probe[i]];
    printf("  L%02d kind=%d stride=%d cin=%3d cout=%3d x_zp=%4d y_zp=%4d xs=%.6f\n",
           probe[i], (int)L.kind, (int)L.stride, (int)L.cin, (int)L.cout,
           (int)L.x_zp, (int)L.y_zp, (double)L.x_scale);
  }
  if (KPT_LAYER_COUNT != 52) printf("  !! KPT_LAYER_COUNT = %d（应为 52）\n", KPT_LAYER_COUNT);

  printf("=== 读入权重与输入 ===\n");
  if (!load("data/kpt_weights.bin", g_blob, KPT_BLOB_BYTES)) return 2;
  if (!load("build/input.bin", g_img, (int)sizeof(g_img))) return 2;

  printf("=== 跑整网 ===\n");
  kpt_net_entry(g_blob, g_img, g_scratch, g_hm, g_pres, g_hand);

  int mn = 255, mx = 0;
  long long sum = 0;
  int n = (int)sizeof(g_hm);
  for (int i = 0; i < n; ++i) {
    int v = g_hm[i];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    sum += v;
  }
  printf("  hm     : 21x12x12  min %d  max %d  mean %.2f\n", mn, mx, (double)sum / n);
  printf("  pres   : %d\n", (int)g_pres[0]);
  printf("  hand   : [%d,%d]\n", (int)g_hand[0], (int)g_hand[1]);
  printf("  （g++ 版正确结果示例：hm mean 约 130~160，pres 约 200~245）\n");
  return 0;
}
