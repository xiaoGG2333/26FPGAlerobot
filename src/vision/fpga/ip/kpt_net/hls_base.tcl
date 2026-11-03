# 单层基线综合（只为快速拿真实资源/时序数字，不参与上板）
#
# 综合两个代表性单层，各自一个 solution / 一份 csynth.rpt：
#   solution pw : kpt_base_pw  逐点 1x1 16->96 @96x96 （最重的一层，14.16M MACs）
#   solution dw : kpt_base_dw  逐通道 3x3 96ch @96x96 s2 -> 48x48
#
# 注意：2018.3 里 set_part / create_clock 必须在 open_solution 之后，
#       否则报 [HLS 200-71] set_part is ignored（没有活动 solution）。
#
# 用法（cmd，在 fpga/ip/kpt_net 目录下）：
#   call D:\FPGA\vivado\Vivado\2018.3\settings64.bat
#   set XILINXD_LICENSE_FILE=D:\FPGA\licence\vivadoLicence.lic
#   vivado_hls -f hls_base.tcl -l hls_base.log

proc stage {msg} {
  puts "\n===== [clock format [clock seconds] -format %H:%M:%S] $msg ====="
  flush stdout
}

stage "1/5 建工程 + 读源码"
open_project -reset kpt_base_prj
add_files src/kpt_base.cpp -cflags "-I data -I src"

stage "2/5 综合逐点层 kpt_base_pw（16->96 @96x96）"
set_top kpt_base_pw
open_solution -reset pw
set_part xc7z100ffg900-2
create_clock -period 8
csynth_design
puts "  报告：kpt_base_prj/pw/syn/report/kpt_base_pw_csynth.rpt"

stage "3/5 综合逐通道层 kpt_base_dw（96ch @96x96 s2）"
set_top kpt_base_dw
open_solution -reset dw
set_part xc7z100ffg900-2
create_clock -period 8
csynth_design
puts "  报告：kpt_base_prj/dw/syn/report/kpt_base_dw_csynth.rpt"

stage "4/5 完成"
stage "5/5 结束"
