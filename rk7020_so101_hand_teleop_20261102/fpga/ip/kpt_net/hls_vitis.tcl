# Vitis HLS 2020.2+ 版本（本机没装，留作将来迁移用）
open_project -reset kpt_net_prj
set_top kpt_net_entry
add_files src/kpt_hls.cpp -cflags "-I data -I src"
add_files src/kpt_net.cpp  -cflags "-I data -I src"
add_files -tb tb/tb_kpt_hls.cpp -cflags "-I data -I src"
open_solution -reset solution1 -flow_target vivado
set_part xc7z100ffg900-2
create_clock -period 8 -name default
csynth_design
# export_design -format ip_catalog -output ./export
exit
