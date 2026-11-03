# Vivado HLS 2018.3 综合脚本（本机已装：D:\FPGA\vivado\Vivado\2018.3）
#
# 运行（cmd）：
#   D:\FPGA\vivado\Vivado\2018.3\settings64.bat
#   set XILINXD_LICENSE_FILE=D:\FPGA\licence\vivadoLicence.lic
#   vivado_hls -f hls.tcl 2>&1 | tee ..\..\..\_scratch\hls.log
#
# 说明：vivado_hls 没有进度条，只有分阶段 INFO 日志。
#   本脚本在每个阶段前打一行带时间戳的标记，方便用 tail 看进展。

proc stage {msg} {
  set t [clock format [clock seconds] -format %H:%M:%S]
  puts "\n===== \[$t\] $msg ====="
  flush stdout
}

stage "1/6 建工程 + 读源码"
open_project -reset kpt_net_prj
set_top kpt_net_entry
add_files src/kpt_hls.cpp -cflags "-I data -I src"

stage "2/6 建 solution + 器件/时钟约束（xc7z100ffg900-2 @ 8ns）"
open_solution -reset solution1
set_part xc7z100ffg900-2
create_clock -period 8 -name default

stage "3/6 C 综合开始（调度→绑定→RTL 生成，最慢的一段）"
csynth_design

stage "4/6 综合完成"
puts "  资源/时序报告：kpt_net_prj/solution1/syn/report/csynth.rpt"
puts "  日志：kpt_net_prj/solution1/syn/report/*.rpt 与 vivado_hls.log"

stage "5/6 导出 IP（需要时手动取消注释）"
# export_design -format ip_catalog -output ./export

stage "6/6 结束"
exit
