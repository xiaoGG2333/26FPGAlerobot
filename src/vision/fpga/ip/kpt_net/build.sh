#!/bin/sh
# 一条命令重建全部本地对拍程序
#   用法：./build.sh
CXX=${CXX:-g++}
FLAGS="-O2 -std=c++17 -I data -I src -Wno-unknown-pragmas"
mkdir -p build
set -e
echo "[1/4] 功能模型对拍台 tb_kpt_net"
$CXX $FLAGS tb/tb_kpt_net.cpp src/kpt_net.cpp src/kpt_hls.cpp -o build/tb_kpt_net.exe
echo "[2/4] HLS 结构版对拍台 tb_kpt_hls"
$CXX $FLAGS tb/tb_kpt_hls.cpp src/kpt_net.cpp src/kpt_hls.cpp src/kpt_hls_gen.cpp -o build/tb_kpt_hls.exe
echo "[3/4] 逐层跟踪 tb_trace（功能模型）"
$CXX $FLAGS -DKPT_TRACE tb/tb_kpt_net.cpp src/kpt_net.cpp src/kpt_hls.cpp -o build/tb_trace.exe
echo "[4/4] 逐层跟踪 tb_hlstrace（HLS 结构版 + 功能模型两路）"
$CXX $FLAGS -DKPT_TRACE -DKPT_HLS_TRACE tb/tb_kpt_hls.cpp src/kpt_net.cpp src/kpt_hls.cpp src/kpt_hls_gen.cpp -o build/tb_hlstrace.exe
echo "完成 -> build/"
