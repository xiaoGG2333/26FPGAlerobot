#include "gripper_control.h"
static grip_controller grip;
static int grip_initialized=0;
static void grip_setup(void){if(!grip_initialized){grip_init(&grip,&servo_bus);grip_initialized=1;}}
static void grip_reply(int rc){
 char text[240];snprintf(text,sizeof text,"GRIP1 STATE rc=%d active=%d frozen=%d valid=%d actual=%d goal=%d direction=%d torque_off=%d\r\n",rc,grip.active,grip.frozen,grip.valid,grip.actual,grip.goal,grip.direction,grip.off_confirmed);send_text(text);
}
static void grip_guard_result(int rc){
 if(rc==GP_TEMPERATURE){arm_stop(&arm,ARM_TEMPERATURE);grip_end(&grip);}
 else if(rc&&grip.active&&grip.valid){grip_freeze(&grip,rc);if(arm.active&&!arm.frozen)arm_fault(&arm,rc==GP_WATCHDOG?ARM_WATCHDOG:rc==GP_LIMIT?ARM_Q_LIMIT:rc);}
}
static void grip_service(void){
 if(!grip_initialized||!grip.active)return;
 if(!arm.active){grip_end(&grip);return;}
 if((arm.frozen||!chip_motion_allowed())&&!grip.frozen)grip_freeze(&grip,GP_WATCHDOG);
 int rc=grip_poll(&grip);grip_guard_result(rc);
}
static void grip_command(char **t,int n){
 grip_setup();int rc=ST_ARGUMENT;
 if(n==7&&!strcmp(t[1],"CONFIG")){
  unsigned long v[5];for(int i=0;i<5;i++)if(!pose_uint(t[i+2],&v[i])||v[i]>4095){grip_reply(ST_ARGUMENT);return;}
  rc=grip_config(&grip,(int)v[0],(int)v[1],(unsigned)v[2],(unsigned)v[3],(unsigned)v[4]);
 }else if(n==2&&!strcmp(t[1],"ENABLE")){
  if(!arm.active||arm.frozen||!chip_motion_allowed()||!servo_ready)rc=GP_NOT_ENABLED;
  else rc=grip_begin(&grip);
 }else if(n==3&&!strcmp(t[1],"INPUT")){
  int direction=!strcmp(t[2],"CLOSE")?-1:!strcmp(t[2],"OPEN")?1:!strcmp(t[2],"HOLD")?0:2;
  if(!arm.active||arm.frozen||!chip_motion_allowed()){if(grip.active)grip_freeze(&grip,GP_WATCHDOG);rc=GP_NOT_ENABLED;}
  else rc=grip_input(&grip,direction);
 }else if(n==2&&!strcmp(t[1],"RESUME")){
  if(!arm.active||arm.frozen||!chip_motion_allowed())rc=GP_NOT_ENABLED;
  else rc=grip_resume(&grip);
 }else if(n==2&&!strcmp(t[1],"STOP"))rc=grip_end(&grip);
 else if(n==2&&!strcmp(t[1],"STATE"))rc=grip.reason;
 grip_guard_result(rc);grip_reply(rc);
}
