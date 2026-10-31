#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "gripper_control.h"
static uint32_t tick;static int actual=2500,temp=30,on=0,goal,cap=500,off_calls,goal_calls,drop,fail_on;
static uint32_t now(void *p){(void)p;return tick;}
static st_bus servo_bus={0,0,0,now};static int servo_ready=1;
int st_read(st_bus *b,unsigned id,unsigned address,unsigned count,uint8_t *data,uint8_t *error){
 (void)b;assert(id==6);*error=0;if(drop)return ST_TIMEOUT;memset(data,0,count);
 if(address==56){data[0]=(uint8_t)actual;data[1]=(uint8_t)(actual>>8);if(count>=15){data[6]=120;data[7]=(uint8_t)temp;}}
 else if(address==9){data[0]=1975&255;data[1]=1975>>8;data[2]=3567&255;data[3]=3567>>8;}
 else if(address==40)data[0]=(uint8_t)on;
 else if(address==48){data[0]=(uint8_t)cap;data[1]=(uint8_t)(cap>>8);}
 else {assert(0);}
 return 0;
}
int st_torque_off(st_bus *b,unsigned id,uint8_t *e){(void)b;assert(id==6);*e=0;off_calls++;if(drop)return ST_TIMEOUT;on=0;return 0;}
int st_torque_on(st_bus *b,unsigned id,uint8_t *e){(void)b;assert(id==6);*e=0;if(fail_on)return ST_TIMEOUT;on=1;return 0;}
int st_cal_write(st_bus *b,unsigned id,unsigned address,unsigned count,const uint8_t *p,uint8_t *e){(void)b;assert(id==6&&address==48&&count==2);*e=0;if(drop)return ST_TIMEOUT;cap=p[0]+(p[1]<<8);return 0;}
int st_set_goal(st_bus *b,unsigned id,unsigned p,unsigned speed,unsigned acceleration,uint8_t *e){(void)b;assert(id==6&&p>=1975&&p<=3567&&speed==50&&acceleration==5);*e=0;if(drop)return ST_TIMEOUT;goal=(int)p;goal_calls++;on=1;return 0;}
static struct {int active,frozen;} arm;
enum {ARM_TEMPERATURE=60,ARM_WATCHDOG=61,ARM_Q_LIMIT=62};
static int arm_faults,arm_stops,chip_ok=1;
static int chip_motion_allowed(void){return chip_ok;}
static void arm_stop(void *unused,int why){(void)unused;assert(why==ARM_TEMPERATURE);arm_stops++;arm.active=0;on=0;}
static void arm_fault(void *unused,int why){(void)unused;(void)why;arm_faults++;arm.frozen=1;}
static void send_text(const char *p){(void)p;}
static int pose_uint(const char *p,unsigned long *v){char *end;*v=strtoul(p,&end,10);return *p&&!*end;}
#include "gripper_uart.inc"
static void reset(void){tick=0;actual=2500;temp=30;on=0;cap=500;off_calls=goal_calls=drop=fail_on=0;arm.active=1;arm.frozen=0;arm_stops=arm_faults=0;chip_ok=1;grip_initialized=0;grip_setup();assert(!grip_config(&grip,2000,3500,100,50,200));}
int main(void){
 reset();assert(!grip_begin(&grip)&&on&&goal==2500&&cap==200);
 assert(!grip_input(&grip,-1));tick=100;grip_service();assert(goal==2490&&on);
 actual=2487;assert(!grip_input(&grip,0)&&goal==2487);int writes=goal_calls;
 actual=2480;assert(!grip_input(&grip,0)&&goal==2487&&goal_calls==writes);
 assert(!grip_input(&grip,1));tick+=100;grip_service();assert(goal==2490);
 int oldgoal=goal,off=off_calls;drop=1;tick+=100;grip_service();assert(grip.active&&grip.valid&&grip.frozen&&arm.frozen&&off_calls==off&&goal==oldgoal);
 drop=0;arm.frozen=0;actual=2470;assert(!grip_resume(&grip)&&goal==2470&&on&&grip.direction==0);
 tick+=501;grip_service();assert(grip.frozen&&on&&off_calls==off);
 reset();assert(!grip_begin(&grip));assert(!grip_input(&grip,-1));tick=100;grip_service();assert(goal==2490);assert(!grip_input(&grip,1)&&goal==2500);tick=200;grip_service();assert(goal==2510);assert(!grip_end(&grip));
 reset();assert(!grip_begin(&grip));temp=56;tick=80;grip_service();assert(grip.pending_temperature&&on);
 tick=1580;grip_input(&grip,0);grip_service();assert(on&&arm_stops==0);
 tick=1661;grip_input(&grip,0);grip_service();assert(!on&&!grip.active&&arm_stops==1&&cap==500);
 reset();fail_on=1;assert(grip_begin(&grip)==ST_TIMEOUT&&!grip.active&&!grip.valid&&!on&&off_calls>0&&cap==500);
 reset();assert(!grip_begin(&grip));arm.active=0;writes=goal_calls;grip_service();assert(!grip.active&&!on&&goal_calls==writes&&cap==500);
 reset();assert(!grip_begin(&grip));on=0;tick=80;grip_service();assert(grip.frozen&&!on&&goal_calls==1);
 reset();actual=1974;assert(grip_begin(&grip)==GP_LIMIT&&!on);
 reset();actual=1999;assert(!grip_begin(&grip)&&goal==1999);assert(!grip_input(&grip,1));tick=20;grip_service();assert(goal>=2000);assert(!grip_end(&grip));
 reset();assert(!grip_config(&grip,3500,2000,100,50,200));assert(!grip_begin(&grip));assert(!grip_input(&grip,-1));tick=100;grip_service();assert(goal==2510);assert(!grip_end(&grip));
 reset();assert(!grip_config(&grip,1900,3500,100,50,200));assert(grip_begin(&grip)==GP_CONFIG&&!on);
 reset();assert(!grip_begin(&grip));char *state[]={"GRIP1","STATE"};grip_command(state,2);
 assert(!grip_end(&grip)&&!on&&cap==500);
 puts("PASS ID6 startup/order/cap, close/open, one-shot actual HOLD, comms/watchdog torque retention, explicit resume, 55C/>1500ms shutdown, STOP cleanup, unexpected torque-off and limits");
 return 0;
}

