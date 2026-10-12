#define main prior_arm_regressions
#include "test_arm_control.c"
#undef main

int main(void)
{
    st_bus b={0,send,receive,now};arm_controller a;
    reset(&a,&b,0,0);assert(arm_begin(&a,3)==0);
    for(int i=0;i<5;i++)assert((unsigned)(memory[i][46]+256U*memory[i][47])==200);
    int delta[5]={200,100,-50,0,25};float q[5];unsigned old_syncs=syncs;
    for(int i=0;i<5;i++)q[i]=delta[i]*so101_right_joints[i].sign/SO101_COUNTS_PER_RAD;
    assert(arm_target(&a,q)==ST_OK&&syncs==old_syncs+1);
    unsigned expected[5]={200,100,50,1,25};
    for(int i=0;i<5;i++){
        assert(a.goal[i]==a.trial_origin[i]+delta[i]&&a.waypoint[i]==a.goal[i]);
        assert((int)(memory[i][42]+256U*memory[i][43])==a.goal[i]);
        assert((unsigned)(memory[i][46]+256U*memory[i][47])==expected[i]);
        assert(memory[i][40]==1);
    }
    assert(memory[5][40]==0);old_syncs=syncs;
    /* No new input: finish existing goal, without replacing it with current pose. */
    for(int n=0;n<4;n++){tick+=80;assert(!arm_keep(&a));arm_poll(&a);assert(a.active&&!a.arrived);}
    assert(syncs==old_syncs);
    for(int i=0;i<5;i++)word(memory[i]+56,a.goal[i]);
    tick+=80;assert(!arm_keep(&a));arm_poll(&a);assert(!a.arrived);
    assert(!arm_target(&a,q)); /* Duplicate does not reset completion. */
    for(int n=0;n<2;n++){tick+=80;assert(!arm_keep(&a));arm_poll(&a);}
    assert(a.arrived&&a.active&&a.mode==3);
    for(int n=0;n<20;n++){tick+=80;assert(!arm_keep(&a));arm_poll(&a);}
    for(int i=0;i<5;i++)assert(memory[i][40]==1);
    assert(syncs==old_syncs);
    q[0]*=.5f;assert(!arm_target(&a,q)&&!a.arrived);
    arm_stop(&a,MOTOR_STOPPED);all_off();

    reset(&a,&b,0,0);assert(arm_begin(&a,3)==0);
    /* Reproduce the old shoulder drift: final goals still reach every motor. */
    word(memory[1]+56,a.trial_origin[1]+18);
    int next[5]={-3,-40,21,1,0};
    for(int i=0;i<5;i++)q[i]=next[i]*so101_right_joints[i].sign/SO101_COUNTS_PER_RAD;
    assert(!arm_target(&a,q));
    assert(a.goal_speeds[1]==200&&a.waypoint[1]==a.trial_origin[1]-40);
    assert(a.waypoint[2]==a.trial_origin[2]+21);
    /* ID3 does not move for >5 seconds: keep goal and torque, not arrival. */
    for(int n=0;n<100&&a.active;n++){
        for(int i=0;i<5;i++)if(i!=2)word(memory[i]+56,a.goal[i]);
        tick+=80;arm_keep(&a);arm_poll(&a);
    }
    assert(a.active&&!a.arrived&&a.outcome==ST_OK);
    for(int i=0;i<5;i++)assert(memory[i][40]==1);
    assert(!arm_hold_current(&a));
    for(int i=0;i<5;i++)assert((unsigned)(memory[i][46]+256U*memory[i][47])==200);
    arm_stop(&a,MOTOR_STOPPED);all_off();
    /* Existing encoder range stop remains in effect on every arm joint. */
    for(int axis=0;axis<5;axis++)for(int side=0;side<2;side++){
        reset(&a,&b,0,0);assert(!arm_begin(&a,3));
        int outside=side?so101_right_joints[axis].maximum+1:so101_right_joints[axis].minimum-1;
        word(memory[axis]+56,outside);tick+=80;assert(!arm_keep(&a));arm_poll(&a);
        assert(!a.active&&a.outcome==MOTOR_HEALTH);all_off();
    }
    reset(&a,&b,0,0);assert(!arm_begin(&a,3));corrupt_goal=1;
    q[2]=.1f;assert(arm_target(&a,q)==ST_FORMAT&&!a.active);all_off();
    reset(&a,&b,0,0);assert(!arm_begin(&a,3));tick+=501;arm_poll(&a);
    assert(!a.active&&a.outcome==ARM_WATCHDOG);all_off();
    reset(&a,&b,0,0);unsigned speeds[5]={200,100,0,1,1};
    int pos[5]={2121,2156,1893,1986,2037};uint8_t error;
    assert(st_sync_arm_goals_speeds(&b,pos,speeds,5,&error)==ST_ARGUMENT&&syncs==0);
    speeds[2]=201;assert(st_sync_arm_goals_speeds(&b,pos,speeds,5,&error)==ST_ARGUMENT&&syncs==0);
    puts("PASS: speed200 startup/hold, proportional broadcast, arrival retains torque, >5s stuck axis stays enabled, ten encoder boundary stops, readback/watchdog cleanup");
    return 0;
}
