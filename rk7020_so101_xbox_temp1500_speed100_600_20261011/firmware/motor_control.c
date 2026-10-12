#include "motor_control.h"
#include "servo_mapping.h"
#include <string.h>
#include <stdlib.h>
static int word(const uint8_t *p){return p[0]+((int)p[1]<<8);}
static int sign_word(const uint8_t *p,int bit)
{int raw=word(p);return raw&(1<<bit)?-(raw&((1<<bit)-1)):raw;}
static int read_motor(motor_controller *c,unsigned id,unsigned addr,unsigned n,uint8_t *p)
{return st_read(c->bus,id,addr,n,p,&c->error);}
static void finish(motor_controller *c,int outcome)
{
    st_trace_freeze();
    c->active=0;c->outcome=outcome;c->done_pending=1;c->off_confirmed=0;
    for(int attempt=0;attempt<3;++attempt){
        if(st_torque_off(c->bus,c->id,&c->error)==ST_OK){c->off_confirmed=1;break;}
    }
    if(!c->off_confirmed&&c->outcome==ST_OK)c->outcome=ST_TX_ERROR;
}
void motor_init(motor_controller *c,st_bus *bus)
{memset(c,0,sizeof *c);c->bus=bus;c->stage="IDLE";}
int motor_jog(motor_controller *c,uint32_t seq,unsigned id,int delta)
{
    static const int eeprom_low[6]={725,815,862,846,147,1975};
    static const int eeprom_high[6]={3458,3208,3078,3215,3996,3567};
    static const int offsets[6]={-1429,-1868,-551,1828,-773,1727};
    uint8_t p[15];int rc;
    if(c->active)return MOTOR_BUSY;
    if(!seq||seq<=c->last_sequence)return MOTOR_STALE;
    int maximum=id==5?240:100;
    if(id<1||id>6||delta < -maximum||delta > maximum||abs(delta)<10)return MOTOR_ARGUMENT;
    unsigned speed=id==5?100U:50U;
    c->error=0;c->stage="PREFLIGHT";st_trace_clear();
    /* Do not move one joint while another is unexpectedly energized. */
    for(unsigned i=1;i<=6;++i){
        rc=read_motor(c,i,40,1,p);if(rc)return rc;
        if(p[0])return MOTOR_TORQUE_ACTIVE;
    }
    rc=read_motor(c,id,9,4,p);if(rc)return rc;
    int low=word(p),high=word(p+2);
    int expected_low=eeprom_low[id-1];
    int expected_high=eeprom_high[id-1];
    if(low!=expected_low||high!=expected_high)return MOTOR_CALIBRATION;
    if(id<=5){low=so101_right_joints[id-1].minimum;high=so101_right_joints[id-1].maximum;}
    rc=read_motor(c,id,31,2,p);if(rc)return rc;
    if(sign_word(p,11)!=offsets[id-1])return MOTOR_CALIBRATION;
    rc=read_motor(c,id,33,1,p);if(rc)return rc;
    if(p[0]!=0)return MOTOR_CALIBRATION;
    rc=read_motor(c,id,56,15,p);if(rc)return rc;
    int origin=sign_word(p,15),target=origin+delta;
    if(origin<low||origin>high||target<low+10||target>high-10)return MOTOR_LIMIT;
    if(p[6]<90||p[6]>126||p[7]>55)return MOTOR_HEALTH;
    c->id=id;c->origin=origin;c->position=origin;c->target=target;
    c->last_sequence=seq;c->stable=0;c->done_pending=0;c->off_confirmed=0;
    /* Establish and read back a current-position goal BEFORE torque enable. */
    c->stage="PREPARE_ORIGIN_GOAL";
    rc=st_set_goal(c->bus,id,(unsigned)origin,speed,5,&c->error);
    if(rc){finish(c,rc);return rc;}
    c->stage="VERIFY_ORIGIN_POSITION";
    rc=read_motor(c,id,56,2,p);
    if(rc||abs(sign_word(p,15)-origin)>5){finish(c,rc?rc:MOTOR_HEALTH);return c->outcome;}
    c->active=1;
    c->stage="ENABLE_TORQUE";
    rc=st_torque_on(c->bus,id,&c->error);
    if(rc){finish(c,rc);return rc;}
    /* Experimental timing separation, not a retry of a failed target write.
     * Keep the verified origin goal while torque enable settles. */
    c->stage="TORQUE_SETTLE";
    uint32_t settle_start=c->bus->milliseconds(c->bus->context);
    unsigned settle_polls=0;
    while((uint32_t)(c->bus->milliseconds(c->bus->context)-settle_start)<150U){
        if(++settle_polls>=30000000U){finish(c,ST_TIMEOUT);return ST_TIMEOUT;}
    }
    c->stage="VERIFY_SETTLED_TORQUE";
    rc=read_motor(c,id,40,1,p);
    if(rc||p[0]!=1){finish(c,rc?rc:MOTOR_HEALTH);return c->outcome;}
    c->stage="VERIFY_SETTLED_POSITION";
    rc=read_motor(c,id,56,15,p);
    if(rc){finish(c,rc);return rc;}
    c->position=sign_word(p,15);
    if(abs(c->position-origin)>5||p[6]<90||p[6]>126||p[7]>55){
        finish(c,MOTOR_HEALTH);return MOTOR_HEALTH;
    }
    c->stage="WRITE_TARGET_GOAL";
    rc=st_set_goal(c->bus,id,(unsigned)target,speed,5,&c->error);
    if(rc){finish(c,rc);return rc;}
    c->stage="FOLLOW";
    c->start_ms=c->bus->milliseconds(c->bus->context);c->last_poll=c->start_ms-80U;
    return ST_OK;
}
void motor_poll(motor_controller *c)
{
    if(!c->active)return;
    uint32_t now=c->bus->milliseconds(c->bus->context);
    if((uint32_t)(now-c->start_ms)>5000U){finish(c,MOTOR_STALLED);return;}
    if((uint32_t)(now-c->last_poll)<80U)return;
    c->last_poll=now;uint8_t p[15];int rc=read_motor(c,c->id,56,15,p);
    if(rc){finish(c,rc);return;}
    c->position=sign_word(p,15);
    if(p[6]<90||p[6]>126||p[7]>55||
       c->position<(c->origin<c->target?c->origin:c->target)-10||
       c->position>(c->origin>c->target?c->origin:c->target)+10){finish(c,MOTOR_HEALTH);return;}
    c->stable=abs(c->position-c->target)<=5&&!p[10]?c->stable+1:0;
    if(c->stable>=3)finish(c,ST_OK);
}
void motor_cancel(motor_controller *c)
{if(c->active)finish(c,MOTOR_STOPPED);}
