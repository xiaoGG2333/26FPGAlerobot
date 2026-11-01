#define main old_test_not_run
#include "test_arm_control.c"
#undef main
static uint32_t fixed_now(void *ctx){(void)ctx;return tick;}
static void on(void){for(int i=0;i<5;i++)assert(memory[i][40]==1);assert(memory[5][40]==0);}
int main(void){
 st_bus b={0,send,receive,now};arm_controller a;float q[5];
 reset(&a,&b,0,0);assert(!arm_begin(&a,3));on();
 for(int i=0;i<5;i++)assert(a.limits.vmax[i]==2000);
 unsigned fast[5]={2000,2000,2000,2000,2000};uint8_t err=0;
 assert(!st_sync_arm_goals_speeds(&b,a.goal,fast,5,&err));
 for(int i=0;i<5;i++){assert(memory[i][46]+256U*memory[i][47]==2000);fast[i]=2001;assert(st_sync_arm_goals_speeds(&b,a.goal,fast,5,&err)==ST_ARGUMENT);fast[i]=2000;}
 int d[5]={200,-300,100,400,50};
 for(int i=0;i<5;i++)q[i]=d[i]*so101_right_joints[i].sign/SO101_COUNTS_PER_RAD;
 unsigned n=syncs;assert(!arm_target(&a,q));assert(syncs==n);
 for(int k=0;k<500;k++){
  tick+=20;assert(!arm_keep(&a));arm_poll(&a);assert(a.active&&!a.frozen);on();
  for(int i=0;i<5;i++)word(memory[i]+56,a.waypoint[i]);
 }
 assert(a.arrived);for(int i=0;i<5;i++)assert(a.waypoint[i]==a.goal[i]);
 for(int i=0;i<5;i++)assert(a.goal_speeds[i]>=200&&a.goal_speeds[i]<=2000);
 int saved[5];memcpy(saved,a.goal,sizeof saved);q[1]=100;
 assert(arm_target(&a,q)==ARM_Q_LIMIT&&!memcmp(saved,a.goal,sizeof saved));on();
 tick+=501;arm_poll(&a);assert(a.active&&a.frozen&&a.outcome==ARM_WATCHDOG);on();
 assert(!arm_resume(&a)&&!a.frozen);on();
 drop_feedback=1;tick+=80;arm_poll(&a);assert(a.frozen&&a.active);on();
 drop_feedback=0;assert(!arm_resume(&a));
 for(int axis=0;axis<5;axis++)for(int side=0;side<2;side++){
  word(memory[axis]+56,side?so101_right_joints[axis].maximum+1:so101_right_joints[axis].minimum-1);
  tick+=80;arm_keep(&a);arm_poll(&a);assert(a.active&&a.frozen);on();
  word(memory[axis]+56,so101_right_joints[axis].zero);assert(!arm_resume(&a));
 }
 b.milliseconds=fixed_now;
 memory[1][63]=56;assert(!arm_refresh(&a));tick+=1500;assert(!arm_refresh(&a));
 memory[1][63]=55;assert(!arm_refresh(&a));memory[1][63]=56;assert(!arm_refresh(&a));
 tick+=1501;arm_keep(&a);arm_poll(&a);assert(!a.active&&a.outcome==ARM_TEMPERATURE);all_off();
 reset(&a,&b,0,0);assert(!arm_begin(&a,3));arm_stop(&a,MOTOR_STOPPED);all_off();
 b.milliseconds=now;reset(&a,&b,0,0);fail_cap_write=1;assert(arm_begin(&a,3)!=0);all_off();
 puts("PASS S-curve controller, final goal hold/torque, watchdog/comms/range freeze, resume, unchanged servo temperature filter, STOP/startup off");
}

