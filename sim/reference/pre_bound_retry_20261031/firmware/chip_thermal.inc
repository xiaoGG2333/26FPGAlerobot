/* Read-only AXI GPIO fed by autonomous XADC/fan hardware. */
#include "chip_guard.h"
#define CHIP_GPIO_BASE 0x41200000U
static chip_guard chip={.latched=1,.fan_pct=100};
static uint32_t chip_rpm=0,chip_last_poll=0,chip_last_emit=0;
static int chip_thermal_started=0;
static int chip_last_abnormal=-1;
static void chip_read(void){
 chip_guard_update_at(&chip,Xil_In32(CHIP_GPIO_BASE),servo_ms(0));
 chip_rpm=Xil_In32(CHIP_GPIO_BASE+8U);
}
static int chip_motion_allowed(void){chip_read();return chip.allowed;}
static void chip_thermal_emit(unsigned long seq){
 char line[200];chip_read();
 const char *state=!chip.valid?"SENSOR_FAULT":!chip.allowed?"HOT_HOLD":chip.fan_pct==100?"FULL":"NORMAL";
 if(chip.valid)snprintf(line,sizeof line,"THERMAL1 %lu chip_c=%.2f fan_pct=%d rpm=%lu state=%s sensor_ok=1\r\n",seq,(double)chip.celsius,chip.fan_pct,(unsigned long)chip_rpm,state);
 else snprintf(line,sizeof line,"THERMAL1 %lu chip_c=nan fan_pct=100 rpm=%lu state=SENSOR_FAULT sensor_ok=0\r\n",seq,(unsigned long)chip_rpm);
 send_text(line);chip_last_emit=servo_ms(0);
}
static void chip_thermal_service(void){
 if(!chip_thermal_started)return;
 uint32_t now=servo_ms(0);
 if((uint32_t)(now-chip_last_poll)<100U)return;
 chip_last_poll=now;chip_read();
 if(!chip.allowed && arm.active && (!arm.frozen || arm.outcome!=ARM_CHIP_THERMAL))arm_fault(&arm,ARM_CHIP_THERMAL);
 int abnormal=!chip.allowed;
 if(abnormal && (chip_last_abnormal!=abnormal || (uint32_t)(now-chip_last_emit)>=2000U))chip_thermal_emit(pose_last_sequence);
 if(chip_last_abnormal==1 && !abnormal)send_text("CHIP1 RECOVERED RESUME_REQUIRED\r\n");
 chip_last_abnormal=abnormal;
}
static void chip_thermal_init(void){
 chip_thermal_started=1;
 // Wait for a valid, cool sample before running the original startup self-tests.
 while(!chip_motion_allowed()){chip_thermal_emit(0);usleep(100000);}
 chip_last_abnormal=0;
}
