#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../firmware/arm_control.h"
#include "../firmware/arm_alignment.h"
#include "../firmware/servo_mapping.h"
#include "../firmware/motor_control.h"
static uint8_t memory[6][80],reply[80];
static unsigned used,len,tick,writes,enables,syncs;
static int drop_feedback,corrupt_goal,stuck_off;
static int simulate_goal_enable,drop_cap_reads,move_during_caps,fail_cap_write,drop_after_goal;
static const int low[6]={725,815,862,846,147,1975};
static const int high[6]={3458,3208,3078,3215,3996,3567};
static const int offsets[6]={-1429,-1868,-551,1828,-773,1727};
static void word(uint8_t *p,int n){p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static uint32_t now(void *c){(void)c;return tick++;}
static int receive(void *c,uint8_t *b){(void)c;if(used==len)return 0;*b=reply[used++];return 1;}
static int send(void *c,const uint8_t *p,unsigned count)
{
    (void)c;unsigned sum=0;for(unsigned i=2;i<count;++i)sum+=p[i];assert((sum&255)==255);
    len=used=0;
    if(p[2]==254){
        assert(count==48&&p[3]==44&&p[4]==0x83&&p[5]==41&&p[6]==7);syncs++;
        if(simulate_goal_enable)for(int i=0;i<5;i++)assert((unsigned)(memory[i][48]+256U*memory[i][49])<=RK_TRIAL_TORQUE);
        for(int i=0;i<5;++i){
            unsigned o=7+8*(unsigned)i;assert(p[o]==i+1&&p[o+1]<=5&&(unsigned)(p[o+6]+256U*p[o+7])<=RK_MANUAL_SPEED);
            if(corrupt_goal&&i==2)continue;
            memcpy(memory[i]+41,p+o+1,7);
            if(simulate_goal_enable)memory[i][40]=1;
        }
        if(drop_after_goal)drop_feedback=1;
        return (int)count;
    }
    assert(p[2]>=1&&p[2]<=6);uint8_t *m=memory[p[2]-1];
    if(p[4]==3){
        writes++;
        if(p[5]==48){
            assert(count==9&&p[2]<=5);m[48]=p[6];m[49]=p[7];
            if(fail_cap_write&&p[2]==3&&(unsigned)(p[6]+256U*p[7])==RK_TRIAL_TORQUE)drop_cap_reads=3;
            if(move_during_caps&&p[2]==5){word(memory[3]+56,so101_right_joints[3].zero+10);move_during_caps=0;}
        }else{
            assert(p[5]==40&&count==8);
            if(p[6]){assert(p[2]<=5);assert(m[42]==m[56]&&m[43]==m[57]);enables++;}
            if(!(stuck_off&&!p[6]))m[40]=p[6];
        }
    }
    if(drop_cap_reads&&p[2]==3&&p[4]==2&&p[5]==48){drop_cap_reads--;return (int)count;}
    if(drop_feedback&&p[2]==3&&p[4]==2&&p[5]==56)return (int)count;
    unsigned n=p[4]==2?p[6]:0;
    reply[len++]=255;reply[len++]=255;reply[len++]=p[2];reply[len++]=(uint8_t)(n+2);reply[len++]=0;
    for(unsigned i=0;i<n;++i)reply[len++]=m[p[5]+i];
    sum=0;for(unsigned i=2;i<len;++i)sum+=reply[i];reply[len++]=(uint8_t)~sum;
    return (int)count;
}
static void reset(arm_controller *a,st_bus *b,int aligned,int geometry)
{
    memset(memory,0,sizeof memory);tick=writes=enables=syncs=used=len=0;drop_feedback=corrupt_goal=stuck_off=0;
    simulate_goal_enable=drop_cap_reads=move_during_caps=fail_cap_write=drop_after_goal=0;
    for(int i=0;i<6;++i){
        word(memory[i]+9,low[i]);word(memory[i]+11,high[i]);
        word(memory[i]+31,offsets[i]<0?(-offsets[i]|2048):offsets[i]);
        word(memory[i]+56,i==5?2428:so101_right_joints[i].zero);
        memory[i][62]=122;memory[i][63]=34;
        word(memory[i]+48,1000);
    }
    arm_init(a,b,aligned,geometry);
}
static void all_off(void){for(int i=0;i<6;++i)assert(memory[i][40]==0);}
int main(void)
{
    st_bus b={0,send,receive,now};arm_controller a;
    reset(&a,&b,0,0);assert(arm_begin(&a,0)==ARM_ALIGN&&writes==0&&syncs==0);
    reset(&a,&b,1,0);assert(arm_begin(&a,0)==ARM_GEOMETRY&&writes==0&&syncs==0);
    reset(&a,&b,1,1);word(memory[1]+56,842);
    assert(arm_begin(&a,0)==ARM_Q_LIMIT&&writes==0&&syncs==0);
    reset(&a,&b,0,0);assert(arm_begin(&a,1)==ST_OK&&a.mode==1&&enables==5&&memory[5][40]==0);
    tick+=501;arm_poll(&a);assert(!a.active&&a.outcome==ARM_WATCHDOG&&a.off_confirmed);all_off();
    reset(&a,&b,1,1);assert(arm_begin(&a,0)==ST_OK&&a.mode==2);
    float target[5];for(int i=0;i<5;++i)target[i]=20/(so101_right_joints[i].sign*SO101_COUNTS_PER_RAD);
    assert(arm_target(&a,target)==ST_OK);
    for(int step=0;step<5;++step){
        assert(arm_keep(&a)==ST_OK);tick+=80;int old[5];memcpy(old,a.waypoint,sizeof old);arm_poll(&a);
        assert(a.active);
        for(int i=0;i<5;++i){assert(a.waypoint[i]-old[i]==4);word(memory[i]+56,a.waypoint[i]);}
    }
    for(int i=0;i<5;++i)assert(a.waypoint[i]==so101_right_joints[i].zero+20);
    arm_stop(&a,MOTOR_STOPPED);assert(!a.active&&a.off_confirmed);all_off();
    reset(&a,&b,1,1);corrupt_goal=1;assert(arm_begin(&a,0)==ST_FORMAT&&enables==0&&a.off_confirmed);all_off();
    reset(&a,&b,1,1);assert(arm_begin(&a,0)==ST_OK);target[0]=NAN;
    assert(arm_target(&a,target)==ARM_Q_LIMIT&&!a.active);all_off();
    reset(&a,&b,1,1);assert(arm_begin(&a,0)==ST_OK);
    for(int i=0;i<5;++i)target[i]=110/(so101_right_joints[i].sign*SO101_COUNTS_PER_RAD);
    assert(arm_target(&a,target)==ARM_TARGET_JUMP&&!a.active);all_off();
    reset(&a,&b,1,1);assert(arm_begin(&a,0)==ST_OK);drop_feedback=1;tick+=80;arm_poll(&a);
    assert(!a.active&&a.outcome==ST_TIMEOUT&&a.off_confirmed);all_off();
    reset(&a,&b,1,1);assert(arm_begin(&a,0)==ST_OK);word(memory[2]+56,so101_right_joints[2].zero+40);
    tick+=80;arm_poll(&a);assert(!a.active&&a.outcome==ARM_TRACKING&&a.off_confirmed);all_off();
    reset(&a,&b,1,1);assert(arm_begin(&a,0)==ST_OK);memory[1][40]=0;tick+=80;arm_poll(&a);
    assert(!a.active&&a.outcome==ST_FORMAT&&a.off_confirmed);all_off();
    reset(&a,&b,1,1);memory[3][63]=56;assert(arm_begin(&a,0)==MOTOR_HEALTH&&enables==0);
    reset(&a,&b,1,1);memory[3][40]=1;assert(arm_begin(&a,0)==MOTOR_TORQUE_ACTIVE&&writes==0);
    reset(&a,&b,1,1);assert(arm_begin(&a,0)==ST_OK);stuck_off=1;arm_stop(&a,MOTOR_STOPPED);
    assert(!a.active&&!a.off_confirmed);
    puts("PASS: unverified mapping/geometry blocked without writes; real Sync Write framing/readback; all current goals before enable; five-axis slew; gripper excluded; hold watchdog; invalid/out-of-range/jump targets; tracking, feedback, torque and health faults; failed shutdown reporting");
    return 0;
}

