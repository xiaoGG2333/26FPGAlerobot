#define main legacy_unused
#include "test_arm_control.c"
#undef main
#include "../firmware/chip_guard.h"
int main(void){st_bus b={0,send,receive,now};arm_controller a;chip_guard g={0};float q[5]={.1f,.05f,0,0,0};int goal[5];
 reset(&a,&b,0,0);assert(!arm_begin(&a,3));assert(!arm_target(&a,q));memcpy(goal,a.goal,sizeof goal);
 chip_guard_update(&g,45300U|(100U<<16)|(1U<<23)|(1U<<24)|(1U<<25));assert(!g.allowed);
 unsigned count=syncs;if(!g.allowed)arm_fault(&a,31);
 assert(a.active&&a.frozen&&a.outcome==31&&syncs==count&&!memcmp(goal,a.goal,sizeof goal));
 for(int i=0;i<5;i++)assert(memory[i][40]==1);
 chip_guard_update(&g,40000U|(100U<<16)|(1U<<23));assert(g.allowed&&a.frozen);
 assert(!arm_resume(&a)&&!a.frozen);for(int i=0;i<5;i++)assert(memory[i][40]==1);
 puts("PASS chip overheat freezes, no torque-off/new goal; cooling alone never resumes; explicit resume retains target");return 0;}
