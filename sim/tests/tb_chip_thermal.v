`timescale 1ns/1ps
module tb_chip_thermal;
reg clk=0;always #5 clk=~clk;
reg resetn=0,strobe=0,ot=0,tach=1;reg [15:0] raw=0;
wire pwm;wire [31:0] status,rpm;
chip_thermal_policy #(.CLK_HZ(100000),.PWM_HZ(1000),.STALE_CYCLES(500)) dut(clk,resetn,strobe,raw,ot,tach,pwm,status,rpm);
task sample(input [15:0] value);begin @(negedge clk);raw=value;strobe=1;@(negedge clk);strobe=0;end endtask
task check(input condition,input [255:0] msg);begin if(!condition)begin $display("FAIL %s",msg);$fatal;end end endtask
integer low_count,i;
initial begin
 repeat(4)@(negedge clk);resetn=1;repeat(2)@(negedge clk);
 check(pwm==0 && status[26],"startup full");
 sample(40000);repeat(2)@(negedge clk);
 low_count=0;for(i=0;i<100;i=i+1)begin @(negedge clk);if(!pwm)low_count=low_count+1;end
 check(low_count==50,"50 percent PWM");check(status[22:16]==50,"normal duty");
 sample(42800);repeat(2)@(negedge clk);check(pwm==0 && status[22:16]==100,"55C full");
 sample(42500);repeat(2)@(negedge clk);check(status[22:16]==100,"hysteresis");
 sample(41800);repeat(2)@(negedge clk);check(status[22:16]==50,"50C recovery");
 sample(45300);repeat(2)@(negedge clk);check(status[25] && pwm==0,"60C hot");
 sample(40000);repeat(501)@(negedge clk);check(status[26] && pwm==0,"stale full");
 sample(0);repeat(2)@(negedge clk);check(status[26] && pwm==0,"invalid full");
 sample(40000);ot=1;repeat(2)@(negedge clk);check(status[27] && pwm==0,"hardware OT full");
 ot=0;
 for(i=0;i<4;i=i+1)begin tach=0;repeat(10)@(negedge clk);tach=1;repeat(10)@(negedge clk);end
 repeat(100001)@(negedge clk);check(rpm==120,"tach 4 pulses = 120 RPM");
 $display("PASS autonomous PWM/threshold/hysteresis/stale/sensor/OT/tach RTL");$finish;
end
endmodule
