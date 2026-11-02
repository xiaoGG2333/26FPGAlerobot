/* Exclusive, bounded calibration transport. No instruction retry on motion writes. */
#include "cal_bridge.h"
#include <string.h>
#include <stdlib.h>
static unsigned word(const uint8_t *p){return p[0]+((unsigned)p[1]<<8);}
static int read_reg(cal_controller *c,unsigned id,unsigned a,unsigned n,uint8_t *p)
{return st_read(c->bus,id,a,n,p,&c->error);}
static int write_reg(cal_controller *c,unsigned id,unsigned a,unsigned n,const uint8_t *p)
{return st_cal_write(c->bus,id,a,n,p,&c->error);}
static int health(cal_controller *c,unsigned id)
{
    uint8_t p[2];int rc=read_reg(c,id,62,2,p);if(rc)return rc;
    return p[0]<90||p[0]>126||p[1]>55?CAL_HEALTH:ST_OK;
}
void cal_init(cal_controller *c,st_bus *bus)
{memset(c,0,sizeof *c);c->bus=bus;}
int cal_begin(cal_controller *c)
{
    if(c->active)return CAL_BUSY;
    if(c->fault)return CAL_FAULT;
    st_trace_clear();
    uint8_t p[1];c->snapshot_valid=0;c->error=0;
    for(unsigned id=1;id<=6;++id){
        int rc=read_reg(c,id,40,1,p);if(rc)return rc;
        if(p[0])return CAL_TORQUE_ACTIVE;
        rc=health(c,id);if(rc)return rc;
        rc=read_reg(c,id,0,32,c->saved[id-1]);if(rc)return rc;
        rc=read_reg(c,id,32,8,c->saved[id-1]+32);if(rc)return rc;
        rc=read_reg(c,id,55,1,c->saved_lock+id-1);if(rc)return rc;
        const uint8_t *saved=c->saved[id-1];
        if(c->saved_lock[id-1]>1||saved[33]!=0||word(saved+3)!=777||saved[5]!=id||
           word(saved+9)>=word(saved+11)||word(saved+11)>4095)return CAL_FORBIDDEN;
    }
    c->snapshot_valid=1;c->active=1;c->last_seq=0;
    c->moving_id=0;
    c->first_fail_id=0;c->first_fail_addr=0;c->first_fail_rc=0;
    memset(c->goal_ready,0,sizeof c->goal_ready);
    c->off_confirmed=1;c->rollback_success=0;c->event_pending=0;
    c->last_rpc=c->bus->milliseconds(c->bus->context);return ST_OK;
}
static int touch(cal_controller *c)
{
    if(!c->active)return c->fault?CAL_FAULT:CAL_INACTIVE;
    uint32_t now=c->bus->milliseconds(c->bus->context);
    if((uint32_t)(now-c->last_rpc)>CAL_WATCHDOG_MS){
        cal_finish(c,0);c->event_pending=1;return CAL_INACTIVE;
    }
    c->last_rpc=now;return ST_OK;
}
int cal_keep(cal_controller *c){return touch(c);}
static int sequence(cal_controller *c,uint32_t seq)
{
    int rc=touch(c);if(rc)return rc;
    if(!seq||seq<=c->last_seq)return CAL_STALE;
    c->last_seq=seq;return ST_OK;
}
static int fail(cal_controller *c,int rc)
{cal_finish(c,0);return rc;}
int cal_read(cal_controller *c,uint32_t seq,unsigned id,unsigned a,unsigned n,uint8_t *p)
{
    int rc=sequence(c,seq);if(rc)return rc;
    if(id<1||id>6||n<1||n>32||a>79||a+n>80||!p)return CAL_FORBIDDEN;
    rc=read_reg(c,id,a,n,p);return rc?fail(c,rc):ST_OK;
}
static int permitted(unsigned a,unsigned n,const uint8_t *p)
{
    unsigned v=n==2?word(p):p[0];
    switch(a){
    case 9:case 11:case 42:return n==2&&v<=4095;
    case 16:case 48:return n==2&&v<=600;
    case 31:return n==2&&(v&~4095U)==0;
    case 33:case 40:case 55:return n==1&&v<=1;
    case 34:return n==1&&v<=80;
    case 35:return n==1&&v<=200;
    case 36:return n==1&&v<=95;
    case 41:return n==1&&v<=10;
    case 46:return n==2&&(v&32767U)<=400;
    default:return 0;
    }
}
static int settings_guard(cal_controller *c,unsigned id,uint8_t *mode_out)
{
    uint8_t p[2],mode;int rc=health(c,id);if(rc)return rc;
    rc=read_reg(c,id,16,2,p);if(rc)return rc;
    if(word(p)>600)return CAL_ENABLE_GUARD;
    rc=read_reg(c,id,48,2,p);if(rc)return rc;
    if(word(p)>600)return CAL_ENABLE_GUARD;
    rc=read_reg(c,id,41,1,p);if(rc)return rc;
    if(p[0]>10)return CAL_ENABLE_GUARD;
    rc=read_reg(c,id,33,1,&mode);if(rc)return rc;
    rc=read_reg(c,id,46,2,p);if(rc)return rc;
    if((word(p)&32767U)>400)return CAL_ENABLE_GUARD;
    if(mode>1)return CAL_ENABLE_GUARD;
    if(mode_out)*mode_out=mode;
    return ST_OK;
}
static int close_to_current(cal_controller *c,unsigned id,unsigned goal)
{
    uint8_t p[4];int rc=read_reg(c,id,9,4,p);if(rc)return rc;
    unsigned low=word(p),high=word(p+2);
    if(low>=high||high>4095||goal<low||goal>high)return CAL_ENABLE_GUARD;
    rc=read_reg(c,id,56,2,p);if(rc)return rc;
    unsigned current=word(p);
    if(current>4095||current<low||current>high||abs((int)goal-(int)current)>5)return CAL_ENABLE_GUARD;
    return ST_OK;
}
static int enable_guard(cal_controller *c,unsigned id)
{
    uint8_t p[2],mode;int rc=settings_guard(c,id,&mode);if(rc)return rc;
    rc=read_reg(c,id,46,2,p);if(rc)return rc;
    if(mode==1)return (word(p)&32767U)?CAL_ENABLE_GUARD:ST_OK;
    if(mode!=0)return CAL_ENABLE_GUARD;
    if(!c->goal_ready[id-1])return CAL_ENABLE_GUARD;
    rc=read_reg(c,id,42,2,p);if(rc)return rc;
    return close_to_current(c,id,word(p));
}
int cal_hold(cal_controller *c,uint32_t seq,unsigned id)
{
    int rc=sequence(c,seq);if(rc)return c->active?fail(c,rc):rc;
    if(id<1||id>6)return fail(c,CAL_FORBIDDEN);
    uint8_t p[4],position[2],one=1;
    rc=read_reg(c,id,33,1,p);if(rc)return fail(c,rc);
    if(p[0]!=0)return fail(c,CAL_ENABLE_GUARD);
    rc=read_reg(c,id,40,1,p);if(rc)return fail(c,rc);
    if(p[0])return fail(c,CAL_TORQUE_ACTIVE);
    rc=read_reg(c,id,46,2,p);if(rc)return fail(c,rc);
    if(word(p))return fail(c,CAL_ENABLE_GUARD);
    /* This servo firmware auto-enables on Goal_Position. Check limits first. */
    rc=settings_guard(c,id,0);if(rc)return fail(c,rc);
    rc=read_reg(c,id,9,4,p);if(rc)return fail(c,rc);
    unsigned low=word(p),high=word(p+2);
    if(low>=high||high>4095)return fail(c,CAL_ENABLE_GUARD);
    rc=read_reg(c,id,56,2,position);if(rc)return fail(c,rc);
    unsigned current=word(position);
    if(current>4095||current<low||current>high)return fail(c,CAL_ENABLE_GUARD);
    /* Read current, replace and verify the old goal, then enable within this RPC.
       A moving axis must still satisfy the unchanged five-count guard. */
    c->off_confirmed=0;
    rc=write_reg(c,id,42,2,position);if(rc)return fail(c,rc);
    c->goal_ready[id-1]=1;
    rc=enable_guard(c,id);if(rc)return fail(c,rc);
    rc=read_reg(c,id,40,1,p);if(rc)return fail(c,rc);
    if(p[0]==0){rc=write_reg(c,id,40,1,&one);if(rc)return fail(c,rc);}
    else if(p[0]!=1)return fail(c,CAL_ENABLE_GUARD);
    return ST_OK;
}
int cal_write(cal_controller *c,uint32_t seq,unsigned id,unsigned a,unsigned n,const uint8_t *p)
{
    int rc=sequence(c,seq);if(rc)return rc;
    if(id<1||id>6||!p||n<1||n>2||!permitted(a,n,p))return CAL_FORBIDDEN;
    uint8_t q[2],mode;
    if(a<40){
        rc=read_reg(c,id,40,1,q);if(rc)return fail(c,rc);
        if(q[0])return CAL_TORQUE_ACTIVE;
        rc=read_reg(c,id,55,1,q);if(rc)return fail(c,rc);
        if(q[0])return CAL_ENABLE_GUARD;
    }
    if(a==40&&p[0]){
        rc=enable_guard(c,id);if(rc)return rc<=6||rc==CAL_HEALTH?fail(c,rc):rc;
    }
    if(a==42){
        rc=settings_guard(c,id,&mode);if(rc)return fail(c,rc);
        if(mode!=0)return fail(c,CAL_ENABLE_GUARD);
        rc=close_to_current(c,id,word(p));if(rc)return fail(c,rc);
    }
    if(a==46&&!(word(p)&32767U)){
        rc=read_reg(c,id,33,1,&mode);if(rc)return fail(c,rc);
        if(mode==1){rc=settings_guard(c,id,0);if(rc)return fail(c,rc);}
    }
    if(a==46&&(word(p)&32767U)){
        rc=settings_guard(c,id,&mode);if(rc)return fail(c,rc);
        if(mode!=1)return CAL_ENABLE_GUARD;
        rc=read_reg(c,id,40,1,q);if(rc)return fail(c,rc);
        if(q[0]!=1)return CAL_ENABLE_GUARD;
        for(unsigned other=1;other<=6;++other)if(other!=id){
            rc=read_reg(c,other,33,1,&mode);if(rc)return fail(c,rc);
            if(mode==1){rc=read_reg(c,other,46,2,q);if(rc)return fail(c,rc);
                if(word(q)&32767U)return CAL_SECOND_VELOCITY;}
        }
    }
    /* Mark before transmission: lost ACK/readback must still stop this axis first. */
    if(a==46&&(word(p)&32767U))c->moving_id=id;
    if(a==42||a==46)c->off_confirmed=0;
    rc=write_reg(c,id,a,n,p);
    if(rc)return fail(c,rc);
    if(a==31||a==33)c->goal_ready[id-1]=0;
    if(a==42)c->goal_ready[id-1]=1;
    if(a==40&&p[0])c->off_confirmed=0;
    if(a==46&&!(word(p)&32767U)&&c->moving_id==id)c->moving_id=0;
    return ST_OK;
}
static void cleanup_failure(cal_controller *c,unsigned id,unsigned a,int rc)
{if(rc&&!c->first_fail_rc){c->first_fail_id=id;c->first_fail_addr=a;c->first_fail_rc=rc;}}
int cal_finish(cal_controller *c,int commit)
{
    /* Freeze before any stop/rollback traffic; repeated STOP preserves evidence. */
    st_trace_freeze();
    uint8_t zero[2]={0,0},one=1;int off[6]={0},attempts[6]={0};
    if(!c->snapshot_valid){c->active=0;return CAL_INACTIVE;}
    c->active=0;c->off_confirmed=1;c->rollback_success=1;
    c->first_fail_id=0;c->first_fail_addr=0;c->first_fail_rc=0;
    /* Stop the known or uncertain velocity axis before touching unrelated IDs.
       Mode1 Speed=0 can automatically enable torque, so STOP never sends it. */
    if(c->moving_id>=1&&c->moving_id<=6){
        unsigned id=c->moving_id;
        int rc=st_torque_off(c->bus,id,&c->error);cleanup_failure(c,id,40,rc);off[id-1]=rc==ST_OK;
        attempts[id-1]=1;
    }
    /* Distribute retries across all axes; an absent low ID cannot postpone ID6. */
    for(int round=0;round<3;++round)for(unsigned id=1;id<=6;++id){
        if(!off[id-1]&&attempts[id-1]<3){
            int rc=st_torque_off(c->bus,id,&c->error);cleanup_failure(c,id,40,rc);off[id-1]=rc==ST_OK;
            ++attempts[id-1];
        }
    }
    for(unsigned id=1;id<=6;++id){
        if(!off[id-1])c->off_confirmed=0;
    }
    static const unsigned addr[]={9,11,16,31,34,35,36};
    static const unsigned size[]={2,2,2,2,1,1,1};
    for(unsigned id=1;id<=6;++id){
        if(!off[id-1]){c->rollback_success=0;continue;}
        int rc=write_reg(c,id,55,1,zero);cleanup_failure(c,id,55,rc);
        int safe_setup=rc==ST_OK;
        if(safe_setup){
            /* Verified position mode must precede GoalSpeed=0. A failed mode
               readback must never fall through to a possibly Mode1 speed write. */
            rc=write_reg(c,id,33,1,zero);cleanup_failure(c,id,33,rc);safe_setup=rc==ST_OK;
        }
        if(safe_setup){
            rc=write_reg(c,id,46,2,zero);cleanup_failure(c,id,46,rc);safe_setup=rc==ST_OK;
        }
        if(!safe_setup)c->rollback_success=0;
        else {
            if(!commit){
                rc=write_reg(c,id,9,2,zero);cleanup_failure(c,id,9,rc);if(rc)c->rollback_success=0;
                rc=write_reg(c,id,11,2,c->saved[id-1]+11);cleanup_failure(c,id,11,rc);if(rc)c->rollback_success=0;
            }
            for(unsigned j=0;j<sizeof addr/sizeof addr[0];++j){
                if(commit&&(addr[j]==9||addr[j]==11||addr[j]==31))continue;
                rc=write_reg(c,id,addr[j],size[j],c->saved[id-1]+addr[j]);cleanup_failure(c,id,addr[j],rc);
                if(rc!=ST_OK)c->rollback_success=0;
            }
        }
        /* Relock even if unlock/readback failed: the write may have succeeded. */
        rc=write_reg(c,id,55,1,c->saved_lock[id-1]?&one:zero);cleanup_failure(c,id,55,rc);if(rc)c->rollback_success=0;
        uint8_t p=255;rc=read_reg(c,id,40,1,&p);if(!rc&&p)rc=ST_FORMAT;
        cleanup_failure(c,id,40,rc);if(rc)c->off_confirmed=0;
    }
    c->fault=!(c->off_confirmed&&c->rollback_success);
    if(!c->fault)c->moving_id=0;
    if(!c->fault)c->snapshot_valid=0;
    return c->fault?CAL_CLEANUP:ST_OK;
}
void cal_poll(cal_controller *c)
{
    if(c->active&&(uint32_t)(c->bus->milliseconds(c->bus->context)-c->last_rpc)>CAL_WATCHDOG_MS){
        cal_finish(c,0);c->event_pending=1;
    }
}
