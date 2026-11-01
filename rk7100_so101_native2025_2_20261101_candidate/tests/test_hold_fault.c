#define main legacy_tests_not_run
#include "test_arm_control.c"
#undef main
static void on(void){for(int i=0;i<5;i++)assert(memory[i][40]==1);}
static uint32_t fixed_now(void *c){(void)c;return tick;}
int main(void){
 st_bus b={0,send,receive,now};arm_controller a;float q[5]={.1f,.05f,0,0,0};int last[5];
 reset(&a,&b,0,0);assert(!arm_begin(&a,3));assert(RK_MANUAL_SPEED==600);on();
 assert(!arm_target(&a,q));memcpy(last,a.goal,sizeof last);
 for(int i=0;i<5;i++)assert(a.goal_speeds[i]>=100&&a.goal_speeds[i]<=600);
 assert(a.goal_speeds[0]==600);
 q[3]=2;unsigned before=syncs;assert(arm_target(&a,q)==ARM_Q_LIMIT);assert(syncs==before&&!memcmp(last,a.goal,sizeof last));on();
 tick+=501;arm_poll(&a);assert(a.active&&a.frozen&&a.outcome==ARM_WATCHDOG);on();
 q[3]=0;assert(arm_target(&a,q)==ARM_NO_ENABLE);assert(!arm_resume(&a)&&!a.frozen);on();
 drop_feedback=1;tick+=80;arm_poll(&a);assert(a.active&&a.frozen);assert(!memcmp(last,a.goal,sizeof last));on();
 assert(arm_resume(&a)!=0);on();drop_feedback=0;assert(!arm_resume(&a));
 for(int j=0;j<5;j++)for(int side=0;side<2;side++){
  word(memory[j]+56,side?so101_right_joints[j].maximum+1:so101_right_joints[j].minimum-1);tick+=80;arm_poll(&a);assert(a.active&&a.frozen);on();
  word(memory[j]+56,so101_right_joints[j].zero);assert(!arm_resume(&a));
 }
 memory[1][62]=89;tick+=80;arm_poll(&a);assert(a.active&&a.frozen);on();
 memory[1][62]=122;assert(!arm_resume(&a));
 corrupt_goal=1;q[2]=.1f;assert(arm_target(&a,q)!=0&&a.active&&a.frozen);assert(!memcmp(last,a.goal,sizeof last));on();
 corrupt_goal=0;assert(!arm_resume(&a));
 b.milliseconds=fixed_now;
 memory[3][63]=56;assert(!arm_refresh(&a));tick+=1500;
 assert(!arm_refresh(&a)&&a.active);on();
 memory[3][63]=55;assert(!arm_refresh(&a));assert(!a.temperature_pending[3]);
 memory[3][63]=56;assert(!arm_refresh(&a));tick+=1000;
 memory[2][63]=56;assert(!arm_refresh(&a));tick+=500;
 assert(!arm_refresh(&a));assert(a.temperature_since[2]!=a.temperature_since[3]);
 tick+=1;arm_keep(&a);arm_poll(&a);
 assert(!a.active&&a.outcome==ARM_TEMPERATURE);all_off();
 b.milliseconds=now;
 reset(&a,&b,0,0);assert(!arm_begin(&a,3));arm_stop(&a,MOTOR_STOPPED);all_off();
 reset(&a,&b,0,0);fail_cap_write=1;assert(arm_begin(&a,3)!=0&&!a.active);all_off();
 puts("PASS temp1500_speed100_600: proportionate sync, rejected target retains goal, watchdog/communication/ten encoder boundaries/voltage/readback freeze, explicit resume, temperature and startup/STOP off");return 0;
}
