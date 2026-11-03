# 方案 B（逐层展开直写调度）的 Vivado HLS 2018.3 综合脚本
#
# 与 hls.tcl 的区别：
#   hls.tcl     综合 src/kpt_hls.cpp（层表驱动，动态循环边界，工程名 kpt_net_prj）
#   hls_gen.tcl 综合 src/kpt_hls_gen.cpp + src/kpt_hls_gen_top.cpp
#               （52 层全展开，循环边界全是字面量，工程名 kpt_net_gen_prj）
# 两者工程名不同，可以同时存在、互不影响。
#
# 用法（cmd，在 fpga/ip/kpt_net 目录下）：
#   call D:\FPGA\vivado\Vivado\2018.3\settings64.bat
#   set XILINXD_LICENSE_FILE=D:\FPGA\licence\vivadoLicence.lic
#   vivado_hls -f hls_gen.tcl -l hls_gen.log

proc stage {msg} {
  puts "\n===== [clock format [clock seconds] -format %H:%M:%S] $msg ====="
  flush stdout
}

stage "1/6 建工程 + 读源码"
open_project -reset kpt_net_gen_prj
set_top kpt_net_entry
add_files src/kpt_hls_gen_top.cpp -cflags "-I data -I src"
add_files src/kpt_hls_gen.cpp     -cflags "-I data -I src"

stage "2/6 建 solution + 器件/时钟约束（xc7z100ffg900-2 @ 8ns）"
open_solution -reset solution1
set_part xc7z100ffg900-2
create_clock -period 8
config_compile -pipeline_loops 1

stage "3/6 C 综合开始（调度→绑定→RTL 生成，最慢的一段）"
csynth_design

stage "4/6 综合完成"
puts "  资源/时序报告：kpt_net_gen_prj/solution1/syn/report/csynth.rpt"
puts "  日志：hls_gen.log"

stage "5/6 导出 IP（需要时手动取消注释）"
# export_design -format ip_catalog

stage "6/6 结束"
