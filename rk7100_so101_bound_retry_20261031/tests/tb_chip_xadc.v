`timescale 1ns/1ps
module tb_chip_xadc;
reg clk=0;always #4 clk=~clk;
reg resetn=0;
wire pwm;wire [31:0] status,rpm;
chip_thermal_top dut(clk,resetn,1'b1,pwm,status,rpm);
task check(input condition,input [255:0] msg);begin if(!condition)begin $display("FAIL XADC %s status=%h raw=%d",msg,status,status[15:0]);$fatal;end end endtask
initial begin
 #200;resetn=1;
 #399800;check(status[23] && !status[26] && status[22:16]==50,"45C normal");
 #500000;check(status[23] && !status[26] && status[22:16]==100 && status[25],"70C full/hot candidate");
 #500000;check(status[25] && status[22:16]==100,"80C hold");
 #500000;check(status[23] && !status[26] && status[22:16]==100 && !status[25],"55C cooling with hysteresis");
 $display("PASS actual XADC primitive/DRP/16-sample average at 45/70/80/55C");$finish;
end
endmodule
