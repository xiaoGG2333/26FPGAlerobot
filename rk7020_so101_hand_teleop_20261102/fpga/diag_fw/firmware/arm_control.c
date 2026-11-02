#include "arm_control.h"
#include "arm_alignment.h"
#include "servo_mapping.h"
#include "motor_control.h"
#include <stdlib.h>
#include <string.h>
static int word(const uint8_t *p){return p[0]+((int)p[1]<<8);}
void arm_init(arm_controller *a,st_bus *bus,int aligned,int geometry)
{memset(a,0,sizeof *a);a->bus=bus;a->aligned=aligned;a->geometry=geometry;}
int arm_refresh(arm_controller *a)
{
    int temporary[5],fault=ST_OK;uint8_t p[15];
    for(int i=0;i<5;++i){
        int rc=st_read(a->bus,(unsigned)(i+1),56,15,p,&a->error);
        if(rc){a->temperature_pending[i]=0;fault=rc;continue;}
        temporary[i]=word(p);
        if(p[7]>55){
            uint32_t stamp=a->bus->milliseconds(a->bus->context);
            if(!a->temperature_pending[i]){a->temperature_pending[i]=1;a->temperature_since[i]=stamp;}
            else if((uint32_t)(stamp-a->temperature_since[i])>1500U)return ARM_TEMPERATURE;
        }else a->temperature_pending[i]=0;
        const so101_joint_map *m=&so101_right_joints[i];
        if((temporary[i]&32768)||temporary[i]<m->minimum||temporary[i]>m->maximum||p[6]<90||p[6]>126)fault=MOTOR_HEALTH;
    }
    if(fault)return fault;
    memcpy(a->actual,temporary,sizeof temporary);
    return so101_counts_to_q(a->actual,a->q)?ARM_Q_LIMIT:ST_OK;
}

static int configuration(arm_controller *a)
{
    static const int eeprom_low[6]={725,815,862,846,147,1975};
    static const int eeprom_high[6]={3458,3208,3078,3215,3996,3567};
    static const int offsets[6]={-1429,-1868,-551,1828,-773,1727};uint8_t p[4];
    for(int id=1;id<=6;++id){
        int rc=st_read(a->bus,(unsigned)id,40,1,p,&a->error);if(rc)return rc;
        if(p[0])return MOTOR_TORQUE_ACTIVE;
        rc=st_read(a->bus,(unsigned)id,9,4,p,&a->error);if(rc)return rc;
        int low=eeprom_low[id-1];
        int high=eeprom_high[id-1];
        if(word(p)!=low||word(p+2)!=high)return MOTOR_CALIBRATION;
        rc=st_read(a->bus,(unsigned)id,31,2,p,&a->error);if(rc)return rc;
        int raw=word(p),offset=raw&2048?-(raw&2047):raw;
        if(offset!=offsets[id-1])return MOTOR_CALIBRATION;
        rc=st_read(a->bus,(unsigned)id,33,1,p,&a->error);if(rc)return rc;
        if(p[0])return MOTOR_CALIBRATION;
    }
    return ST_OK;
}
int arm_trial_check(arm_controller *a)
{
    if(a->active||a->cap_mask)return MOTOR_BUSY;
    int rc=configuration(a);if(rc)return rc;
    rc=arm_refresh(a);if(rc)return rc;
    int counts[5];if(so101_q_to_counts(a->q,counts))return ARM_Q_LIMIT;
    for(int i=0;i<5;i++)if(a->actual[i]<so101_right_joints[i].minimum+RK_TRIAL_COUNTS+10||a->actual[i]>so101_right_joints[i].maximum-RK_TRIAL_COUNTS-10)return ARM_Q_LIMIT;
    return ST_OK;
}
int arm_manual_check(arm_controller *a)
{
    if(a->active||a->cap_mask)return MOTOR_BUSY;
    int rc=configuration(a);if(rc)return rc;
    rc=arm_refresh(a);if(rc)return rc;
    int counts[5];if(so101_q_to_counts(a->q,counts))return ARM_Q_LIMIT;
    for(int i=0;i<5;i++)if(a->actual[i]<so101_right_joints[i].minimum+10||a->actual[i]>so101_right_joints[i].maximum-10)return ARM_Q_LIMIT;
    return ST_OK;
}
int arm_check(arm_controller *a)
{
    int rc=arm_refresh(a);if(rc)return rc;
    if(!a->aligned)return ARM_ALIGN;
    if(!a->geometry)return ARM_GEOMETRY;
    int counts[5];if(so101_q_to_counts(a->q,counts))return ARM_Q_LIMIT;
    return ST_OK;
}
void arm_fault(arm_controller *a,int outcome)
{
    if(outcome==ARM_TEMPERATURE||!a->valid_goal){arm_stop(a,outcome);return;}
    if(!a->frozen||a->outcome!=outcome)a->event_pending=1;
    a->frozen=1;a->outcome=outcome;a->arrived=0;
    /* No goal or torque writes during a recoverable fault. */
}
int arm_resume(arm_controller *a)
{
    if(!a->active||!a->valid_goal)return ARM_NO_ENABLE;
    int rc=arm_refresh(a);if(rc){arm_fault(a,rc);return rc;}
    for(unsigned id=1;id<=5;id++){
        uint8_t on=0;rc=st_read(a->bus,id,40,1,&on,&a->error);
        if(rc||on!=1){arm_fault(a,rc?rc:ST_FORMAT);return rc?rc:ST_FORMAT;}
    }
    /* Reconcile a lost write ACK against the last verified target first. */
    rc=st_sync_arm_goals_speeds(a->bus,a->goal,a->goal_speeds,5U,&a->error);
    if(rc){arm_fault(a,rc);return rc;}
    memcpy(a->waypoint,a->goal,sizeof a->goal);
    a->frozen=0;a->outcome=ST_OK;a->event_pending=0;
    a->last_command=a->bus->milliseconds(a->bus->context);return ST_OK;
}
void arm_stop(arm_controller *a,int outcome)
{
    a->active=0;a->mode=0;a->valid_goal=0;a->frozen=0;a->arrived=0;a->arrival_samples=0;a->outcome=outcome;a->off_confirmed=1;a->event_pending=1;
    for(unsigned id=1;id<=6;++id){
        int rc=ST_TIMEOUT;
        for(int attempt=0;attempt<3&&rc;++attempt)rc=st_torque_off(a->bus,id,&a->error);
        if(rc)a->off_confirmed=0;
    }
    /* Restore only RAM torque caps after all six torque-off readbacks passed.
       Never restore an old Goal Position: on this hardware it enables torque. */
    if(a->off_confirmed&&a->cap_mask){
        for(int i=0;i<5;i++)if(a->cap_mask&(1U<<i)){
            uint8_t p[2];unsigned id=(unsigned)i+1;
            int rc=st_read(a->bus,id,40,1,p,&a->error);
            if(rc||p[0]){a->off_confirmed=0;break;}
            p[0]=(uint8_t)a->saved_caps[i];p[1]=(uint8_t)(a->saved_caps[i]>>8);
            rc=st_cal_write(a->bus,id,48,2,p,&a->error);
            int off=st_torque_off(a->bus,id,&a->error);
            if(rc||off){a->off_confirmed=0;break;}
            a->cap_mask&=~(1U<<i);
        }
    }
    if(!a->off_confirmed&&outcome==ST_OK)a->outcome=ST_TX_ERROR;
}
int arm_begin(arm_controller *a,int hold_only)
{
    int manual=hold_only==3;
    if(manual)hold_only=2;
    if(a->active||a->cap_mask)return MOTOR_BUSY;
    if(hold_only==0){
        if(!a->aligned)return ARM_ALIGN;
        if(!a->geometry)return ARM_GEOMETRY;
    }
    int rc=configuration(a);if(rc)return rc;
    rc=arm_refresh(a);if(rc)return rc;
    if(hold_only!=1){
        int check[5];if(so101_q_to_counts(a->q,check))return ARM_Q_LIMIT;
        for(int i=0;i<5;++i)if(check[i]<so101_right_joints[i].minimum+10||check[i]>so101_right_joints[i].maximum-10)return ARM_Q_LIMIT;
    }
    if(hold_only==2){
        for(int i=0;i<5;i++){
            uint8_t p[2];unsigned id=(unsigned)i+1;
            rc=st_read(a->bus,id,48,2,p,&a->error);
            if(rc){arm_stop(a,rc);return rc;}
            a->saved_caps[i]=(unsigned)word(p);
            if(!a->saved_caps[i]||a->saved_caps[i]>1000U){arm_stop(a,MOTOR_HEALTH);return MOTOR_HEALTH;}
            unsigned cap=a->saved_caps[i]<RK_TRIAL_TORQUE?a->saved_caps[i]:RK_TRIAL_TORQUE;
            p[0]=(uint8_t)cap;p[1]=(uint8_t)(cap>>8);
            a->cap_mask|=1U<<i; /* include a possibly applied WRITE with lost ACK */
            rc=st_cal_write(a->bus,id,48,2,p,&a->error);
            if(rc){arm_stop(a,rc);return rc;}
        }
        /* All caps now set. Refresh positions immediately before Goal WRITE. */
        rc=arm_refresh(a);if(rc){arm_stop(a,rc);return rc;}
        int check[5];if(!manual&&so101_q_to_counts(a->q,check)){arm_stop(a,ARM_Q_LIMIT);return ARM_Q_LIMIT;}
        for(int i=0;i<5;i++)if(a->actual[i]<so101_right_joints[i].minimum+(manual?0:RK_TRIAL_COUNTS)+10||a->actual[i]>so101_right_joints[i].maximum-(manual?0:RK_TRIAL_COUNTS)-10){arm_stop(a,ARM_Q_LIMIT);return ARM_Q_LIMIT;}
        memcpy(a->trial_origin,a->actual,sizeof a->trial_origin);
    }
    memcpy(a->goal,a->actual,sizeof a->goal);memcpy(a->waypoint,a->actual,sizeof a->waypoint);
    rc=st_sync_arm_goals(a->bus,a->actual,hold_only==2?(manual?RK_MANUAL_SPEED:50U):RK_ARM_SPEED,5U,&a->error);
    if(rc){arm_stop(a,rc);return rc;}
    /* All five old targets have been replaced and verified before any enable. */
    for(unsigned id=1;id<=5;++id){
        uint8_t p[2];rc=st_read(a->bus,id,56,2,p,&a->error);
        if(rc||abs(word(p)-a->actual[id-1])>5){arm_stop(a,rc?rc:MOTOR_HEALTH);return a->outcome;}
        rc=st_torque_on(a->bus,id,&a->error);
        if(rc){arm_stop(a,rc);return rc;}
    }
    a->fault_axis=0;a->manual_session=manual;a->arrived=0;a->arrival_samples=0;
    a->valid_goal=1;a->frozen=0;
    for(int i=0;i<5;i++)a->goal_speeds[i]=RK_MANUAL_SPEED;
    a->active=1;a->mode=hold_only==2?3:hold_only?1:2;a->outcome=ST_OK;a->event_pending=0;a->off_confirmed=0;
    a->last_command=a->bus->milliseconds(a->bus->context);a->last_poll=a->last_command;a->trial_started=a->last_command;
    for(int i=0;i<5;i++){a->progress_ms[i]=a->last_command;a->progress_counts[i]=a->actual[i];}
    return ST_OK;
}
int arm_keep(arm_controller *a)
{
    if(!a->active)return ARM_NO_ENABLE;
    uint32_t now=a->bus->milliseconds(a->bus->context);
    if(a->mode==3&&!a->manual_session&&(uint32_t)(now-a->trial_started)>=RK_TRIAL_MS){arm_fault(a,ARM_TRIAL_TIME);return ARM_TRIAL_TIME;}
    if((uint32_t)(now-a->last_command)>RK_ARM_WATCHDOG_MS){arm_fault(a,ARM_WATCHDOG);return ARM_WATCHDOG;}
    a->last_command=now;return ST_OK;
}
int arm_hold_current(arm_controller *a)
{
    if(!a->active||!a->manual_session)return ARM_NO_ENABLE;
    int rc=arm_refresh(a);if(rc){arm_fault(a,rc);return rc;}
    rc=st_sync_arm_goals(a->bus,a->actual,RK_MANUAL_SPEED,5U,&a->error);
    if(rc){arm_fault(a,rc);return rc;}
    memcpy(a->goal,a->actual,sizeof a->goal);memcpy(a->waypoint,a->actual,sizeof a->waypoint);
    uint32_t now=a->bus->milliseconds(a->bus->context);
    for(int i=0;i<5;i++){a->progress_ms[i]=now;a->progress_counts[i]=a->actual[i];}
    return arm_keep(a);
}
int arm_target(arm_controller *a,const float q[5])
{
    if(!a->active||(a->mode!=2&&a->mode!=3)||a->frozen)return ARM_NO_ENABLE;
    int counts[5];if(so101_q_to_counts(q,counts)){if(a->manual_session){return ARM_Q_LIMIT;}arm_fault(a,ARM_Q_LIMIT);return ARM_Q_LIMIT;}
    for(int i=0;i<5;++i){
        if(counts[i]<so101_right_joints[i].minimum+10||counts[i]>so101_right_joints[i].maximum-10){
            if(a->manual_session){return ARM_Q_LIMIT;}
            arm_fault(a,ARM_Q_LIMIT);return ARM_Q_LIMIT;
        }
        if(a->mode==3&&!a->manual_session&&abs(counts[i]-a->trial_origin[i])>RK_TRIAL_COUNTS){arm_fault(a,ARM_TRIAL_LIMIT);return ARM_TRIAL_LIMIT;}
        if(!a->manual_session&&abs(counts[i]-a->actual[i])>RK_ARM_MAX_TARGET_JUMP){arm_fault(a,ARM_TARGET_JUMP);return ARM_TARGET_JUMP;}
    }
    int rc=arm_keep(a);if(rc)return rc;
    if(a->manual_session && memcmp(a->goal,counts,sizeof counts)){
        /* Refresh measured start; never infer travel from an old model seed. */
        rc=arm_refresh(a);if(rc){arm_fault(a,rc);return rc;}
        int largest=0;
        for(int i=0;i<5;i++){int d=abs(counts[i]-a->actual[i]);if(d>largest)largest=d;}
        unsigned speeds[5];
        for(int i=0;i<5;i++){
            unsigned d=(unsigned)abs(counts[i]-a->actual[i]);
            /* Rounded proportional speeds approximate a common arrival time.
               Motor acceleration/load can still produce unequal feedback. */
            speeds[i]=largest?(d*RK_MANUAL_SPEED+(unsigned)largest/2U)/(unsigned)largest:1U;
            if(speeds[i]<100U)speeds[i]=100U;
        }
        rc=st_sync_arm_goals_speeds(a->bus,counts,speeds,5U,&a->error);
        if(rc){arm_fault(a,rc);return rc;}
        memcpy(a->waypoint,counts,sizeof counts);
        memcpy(a->goal_speeds,speeds,sizeof speeds);
        a->arrived=0;a->arrival_samples=0;
    }
    memcpy(a->goal,counts,sizeof counts);return ST_OK;
}
/* Manual coordinated position control: motor goal persists independently
   of JoyCon input. Neutral input only sends KEEP; never cancels this goal. */
static void manual_goal_poll(arm_controller *a)
{
    int at_goal=1;
    for(int i=0;i<5;i++){
        uint8_t torque=0;int rc=st_read(a->bus,(unsigned)i+1,40,1,&torque,&a->error);
        if(rc||torque!=1){arm_fault(a,rc?rc:ST_FORMAT);return;}
        int gap=abs(a->goal[i]-a->actual[i]);
        if(gap>8)at_goal=0;
        /* No elapsed no-progress shutdown: keep this goal and torque active.
           Encoder range, health and watchdog checks remain in arm_poll. */
    }
    if(at_goal){if(a->arrival_samples<3)a->arrival_samples++;}
    else a->arrival_samples=0;
    a->arrived=a->arrival_samples>=3;
    /* At arrival: no STOP, no torque-off and no replacement of the goal. */
}
void arm_poll(arm_controller *a)
{
    if(!a->active)return;
    if(a->frozen){
        uint32_t tick=a->bus->milliseconds(a->bus->context);
        if((uint32_t)(tick-a->last_poll)>=RK_ARM_SAMPLE_MS){a->last_poll=tick;int rc=arm_refresh(a);if(rc==ARM_TEMPERATURE)arm_stop(a,rc);}
        return;
    }
    uint32_t now=a->bus->milliseconds(a->bus->context);
    if(a->mode==3&&!a->manual_session&&(uint32_t)(now-a->trial_started)>=RK_TRIAL_MS){arm_fault(a,ARM_TRIAL_TIME);return;}
    if((uint32_t)(now-a->last_command)>RK_ARM_WATCHDOG_MS){arm_fault(a,ARM_WATCHDOG);return;}
    if((uint32_t)(now-a->last_poll)<RK_ARM_SAMPLE_MS)return;
    a->last_poll=now;int rc=arm_refresh(a);if(rc){arm_fault(a,rc);return;}
    if(a->manual_session){manual_goal_poll(a);return;}
    int next[5];
    for(int i=0;i<5;++i){
        uint8_t torque=0;rc=st_read(a->bus,(unsigned)(i+1),40,1,&torque,&a->error);
        if(rc||torque!=1){arm_fault(a,rc?rc:ST_FORMAT);return;}
        if(a->mode==3&&!a->manual_session&&abs(a->actual[i]-a->trial_origin[i])>RK_TRIAL_COUNTS+4){arm_fault(a,ARM_TRIAL_LIMIT);return;}
        if(abs(a->actual[i]-a->waypoint[i])>RK_ARM_TRACKING_LIMIT){arm_fault(a,ARM_TRACKING);return;}
        int d=a->goal[i]-a->waypoint[i];
        {
            int step=a->mode==3?1:RK_ARM_STEP_COUNTS;
            if(d>step)d=step;
            if(d<-step)d=-step;
        }
        next[i]=a->waypoint[i]+d;

    }
    if(memcmp(next,a->waypoint,sizeof next)){
        rc=st_sync_arm_goals(a->bus,next,a->manual_session?RK_MANUAL_SPEED:a->mode==3?50U:RK_ARM_SPEED,5U,&a->error);
        if(rc){arm_fault(a,rc);return;}
        memcpy(a->waypoint,next,sizeof next);
    }
}
