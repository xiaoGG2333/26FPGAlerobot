#define main prior_arm_regressions
#include "test_arm_control.c"
#undef main
static void caps_restored(void){for(int i=0;i<5;i++)assert(memory[i][48]==232&&memory[i][49]==3);}
static void target_delta(arm_controller *a,float q[5],int d){for(int i=0;i<5;i++)q[i]=so101_right_joints[i].sign*(a->trial_origin[i]+d-so101_right_joints[i].zero)/SO101_COUNTS_PER_RAD;}
int main(void)
{
    assert(prior_arm_regressions()==0);
    st_bus b={0,send,receive,now};arm_controller a;float q[5];
    reset(&a,&b,0,0);assert(arm_trial_check(&a)==ST_OK&&writes==0&&syncs==0);
    simulate_goal_enable=1;assert(arm_begin(&a,2)==ST_OK&&a.mode==3&&a.aligned==0&&a.geometry==0);
    assert(a.cap_mask==31&&memory[5][40]==0);
    target_delta(&a,q,RK_TRIAL_COUNTS);assert(arm_target(&a,q)==ST_OK);
    int before=a.waypoint[0];assert(arm_keep(&a)==ST_OK);tick+=80;arm_poll(&a);
    assert(a.active&&a.waypoint[0]==before+1&&memory[5][40]==0);
    target_delta(&a,q,RK_TRIAL_COUNTS+1);assert(arm_target(&a,q)==ARM_TRIAL_LIMIT&&!a.active&&a.off_confirmed);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,2)==0);
    tick+=10000;assert(arm_keep(&a)==ARM_TRIAL_TIME&&!a.active);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,2)==0);
    tick+=501;arm_poll(&a);assert(!a.active&&a.outcome==ARM_WATCHDOG);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,2)==0);
    word(memory[2]+56,a.trial_origin[2]+RK_TRIAL_COUNTS+5);tick+=80;arm_poll(&a);
    assert(!a.active&&a.outcome==ARM_TRIAL_LIMIT);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,2)==0);
    drop_feedback=1;tick+=80;arm_poll(&a);assert(!a.active&&a.outcome==ST_TIMEOUT);all_off();caps_restored();
    reset(&a,&b,0,0);move_during_caps=1;simulate_goal_enable=1;assert(arm_begin(&a,2)==0);
    assert(a.trial_origin[3]==so101_right_joints[3].zero+10);
    arm_stop(&a,MOTOR_STOPPED);all_off();caps_restored();
    reset(&a,&b,0,0);word(memory[1]+48,0);assert(arm_begin(&a,2)==MOTOR_HEALTH&&syncs==0);all_off();
    assert(memory[0][48]==232&&memory[0][49]==3&&memory[1][48]==0&&memory[1][49]==0);
    reset(&a,&b,0,0);fail_cap_write=1;
    assert(arm_begin(&a,2)==ST_TIMEOUT&&syncs==0&&a.off_confirmed);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=drop_after_goal=1;
    assert(arm_begin(&a,2)==ST_TIMEOUT&&syncs==1&&a.off_confirmed);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,2)==0);stuck_off=1;arm_stop(&a,MOTOR_STOPPED);
    assert(!a.off_confirmed&&a.cap_mask==31);assert((unsigned)(memory[0][48]+256U*memory[0][49])==RK_TRIAL_TORQUE);
    assert(arm_begin(&a,2)==MOTOR_BUSY);
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    for(int i=0;i<150;i++){tick+=80;assert(arm_keep(&a)==0);arm_poll(&a);assert(a.active);}
    assert(tick-a.trial_started>10000);assert(a.manual_session);
    int old=a.waypoint[0];target_delta(&a,q,60);assert(arm_target(&a,q)==0);tick+=80;assert(arm_keep(&a)==0);arm_poll(&a);assert(a.waypoint[0]==old+8);
    target_delta(&a,q,RK_TRIAL_COUNTS+1);assert(arm_target(&a,q)==ST_OK);
    arm_stop(&a,MOTOR_STOPPED);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    tick+=501;arm_poll(&a);assert(!a.active&&a.outcome==ARM_WATCHDOG);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    arm_stop(&a,MOTOR_STOPPED);all_off();caps_restored();
    assert(arm_begin(&a,2)==0&&!a.manual_session);tick+=10000;
    assert(arm_keep(&a)==ARM_TRIAL_TIME);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    target_delta(&a,q,120);assert(arm_target(&a,q)==0); /* distant final goal accepted */
    for(int n=0;n<5;n++){tick+=80;assert(arm_keep(&a)==0);arm_poll(&a);assert(a.active);}
    assert(a.waypoint[0]==a.trial_origin[0]+16); /* command waits, does not run away */
    for(int n=0;n<100&&a.active;n++){tick+=80;if(a.active)arm_keep(&a);arm_poll(&a);}
    assert(!a.active&&a.outcome==ARM_STALL);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    target_delta(&a,q,120);assert(arm_target(&a,q)==0);
    for(int n=0;n<50;n++){
        for(int i=0;i<5;i++)word(memory[i]+56,a.waypoint[i]);
        tick+=80;assert(arm_keep(&a)==0);arm_poll(&a);assert(a.active);
    }
    assert(a.waypoint[0]==a.trial_origin[0]+120);arm_stop(&a,MOTOR_STOPPED);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;
    word(memory[1]+56,854);word(memory[2]+56,3052);
    assert(arm_manual_check(&a)==ARM_Q_LIMIT);assert(arm_begin(&a,3)==ARM_Q_LIMIT);
    assert(enables==0&&syncs==0);all_off();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    for(int i=0;i<5;i++)q[i]=a.q[i];
    q[2]=2.1f;
    assert(arm_target(&a,q)==ARM_Q_LIMIT&&a.active);
    for(int i=0;i<5;i++){assert(memory[i][40]==1);assert(a.goal[i]==a.actual[i]);}
    assert(memory[5][40]==0);arm_stop(&a,MOTOR_STOPPED);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    a.goal[1]=a.actual[1]+16;a.waypoint[1]=a.actual[1]+5;
    for(int n=0;n<100;n++){tick+=80;assert(arm_keep(&a)==0);arm_poll(&a);assert(a.active);}
    arm_stop(&a,MOTOR_STOPPED);all_off();caps_restored();
    reset(&a,&b,0,0);simulate_goal_enable=1;assert(arm_begin(&a,3)==0);
    a.goal[1]=a.actual[1]+16;a.waypoint[1]=a.actual[1]+5;
    a.goal[2]=a.actual[2]+120;a.waypoint[2]=a.actual[2]+16;
    for(int n=0;n<100&&a.active;n++){tick+=80;arm_keep(&a);arm_poll(&a);}
    assert(!a.active&&a.outcome==ARM_STALL&&a.fault_axis==3);all_off();caps_restored();
    puts("STALL DEAD BAND / EXACT AXIS PASS + OUT HOLD PASS + FEEDBACK FOLLOW PASS + MANUAL SESSION PASS + LIMITED DRIVE PASS: legacy regression + read-only check, caps before auto-enable, fresh origin, slew, total travel, 10s timeout, watchdog, feedback/drift faults, RAM cap restore, gripper exclusion, failed-OFF cap retention");
    return 0;
}
