# HLS 前端自检：只跑 C 仿真，不做综合（很快）
#   vivado_hls -f hls_csim.tcl -l csim.log
#
# 目的：确认 Vivado HLS 的前端能不能正确读自动生成的层表并跑通整网。
# 如果这里打印的层表 / hm 统计是对的，说明"综合出空电路"发生在综合变换阶段；
# 如果这里就不对，说明是 ApCC 前端与 gcc 的差异，得改代码写法。
open_project -reset kpt_net_csim
set_top kpt_net_entry
add_files src/kpt_hls.cpp -cflags "-I data -I src"
add_files -tb tb/tb_csim.cpp -cflags "-I data -I src"
open_solution -reset solution1
set_part xc7z100ffg900-2
puts "\n===== C 仿真开始 ====="
csim_design -clean
puts "===== C 仿真结束 ====="
exit
