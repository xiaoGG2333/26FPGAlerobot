`timescale 1ns/1ps
// Autonomous cooling; FPGA output is inverted by board Q7 (open collector).
module chip_thermal_policy #(
 parameter integer CLK_HZ=125000000, parameter integer PWM_HZ=25000,
 parameter integer STALE_CYCLES=125000000
)(input wire clk,input wire resetn,input wire sample_valid,
 input wire [15:0] sample_raw,input wire overtemp,input wire fan_tach,
 output wire fan_pwm,output wire [31:0] thermal_status,output reg [31:0] fan_rpm);
localparam integer PERIOD=CLK_HZ/PWM_HZ;
reg [31:0] pwm_count,age,window_count;
reg [15:0] raw,pulse_count;
reg valid,full;
(* ASYNC_REG="TRUE" *) reg [2:0] tach_sync;
wire tach_fall=tach_sync[2:1]==2'b10;
wire sensor_fault=!valid || age>=STALE_CYCLES || raw<16'd32920 || raw>16'd51124;
wire hot=valid && raw>16'd43322;
wire force_full=full || hot || sensor_fault || overtemp;
wire [6:0] duty=force_full?7'd100:7'd50;
wire [31:0] pulse_next={16'b0,pulse_count}+(tach_fall?32'd1:32'd0);
wire [31:0] rpm_next=(pulse_next<<5)-(pulse_next<<1); // 2 pulses/rev -> *30
// Low at FPGA pin means high at fan PWM input after Q7 inversion.
assign fan_pwm=force_full?1'b0:~(pwm_count<PERIOD/2);
assign thermal_status={4'b0,overtemp,sensor_fault,hot,force_full,valid,duty,raw};
always @(posedge clk) begin
 if(!resetn) begin
  pwm_count<=0;age<=STALE_CYCLES;raw<=0;valid<=0;full<=1;
  window_count<=0;pulse_count<=0;fan_rpm<=0;tach_sync<=3'b111;
 end else begin
  tach_sync<={tach_sync[1:0],fan_tach};
  pwm_count<=pwm_count==PERIOD-1?0:pwm_count+1;
  if(age<STALE_CYCLES)age<=age+1;
  if(sample_valid) begin
   age<=0;raw<=sample_raw;
   valid<=sample_raw>=16'd32920 && sample_raw<=16'd51124;
   if(sample_raw>=16'd42672)full<=1;
   else if(sample_raw<=16'd42022)full<=0;
  end
  if(window_count==CLK_HZ-1) begin
   window_count<=0;pulse_count<=0;fan_rpm<=rpm_next>65535?65535:rpm_next;
  end else begin
   window_count<=window_count+1;
   if(tach_fall && pulse_count!=65535)pulse_count<=pulse_count+1;
  end
 end
end
endmodule

module chip_thermal_top(input wire clk,input wire resetn,input wire fan_tach,
 output wire fan_pwm,output wire [31:0] thermal_status,output wire [31:0] fan_rpm);
wire [15:0] adc_data;wire adc_ready,eoc,ot;wire [4:0] channel;
reg den,pending;
always @(posedge clk) begin
 if(!resetn) begin den<=0;pending<=0;end
 else begin
  den<=0;
  if(eoc && channel==0 && !pending) begin den<=1;pending<=1;end
  if(adc_ready)pending<=0;
 end
end
// UG480 continuous sequencer: temperature + calibration, average 16 samples.
// DCLK=125MHz, divider=10 -> ADCCLK=12.5MHz. No ARM/USB dependency.
XADC #(.INIT_40(16'h9000),.INIT_41(16'h2ff0),.INIT_42(16'h0a00),
 .INIT_48(16'h0101),.INIT_49(16'h0000),.INIT_4A(16'h0100),.INIT_4B(16'h0000)) adc(
 .DCLK(clk),.RESET(~resetn),.DADDR(7'h00),.DEN(den),.DWE(1'b0),.DI(16'b0),
 .DO(adc_data),.DRDY(adc_ready),.EOC(eoc),.CHANNEL(channel),.OT(ot),
 .CONVST(1'b0),.CONVSTCLK(1'b0),.VP(1'b0),.VN(1'b0),
 .VAUXP(16'b0),.VAUXN(16'b0),.ALM(),.BUSY(),.EOS(),.MUXADDR(),
 .JTAGBUSY(),.JTAGLOCKED(),.JTAGMODIFIED());
chip_thermal_policy policy(.clk(clk),.resetn(resetn),
 .sample_valid(adc_ready && pending),.sample_raw(adc_data),.overtemp(ot),
 .fan_tach(fan_tach),.fan_pwm(fan_pwm),.thermal_status(thermal_status),.fan_rpm(fan_rpm));
endmodule

