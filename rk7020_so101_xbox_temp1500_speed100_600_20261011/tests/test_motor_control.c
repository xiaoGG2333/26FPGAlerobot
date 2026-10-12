#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/motor_control.h"
static unsigned char mem[6][80],reply[80];
static unsigned at,len,tick,writes,enables,enable_tick,target_writes;
static int no_write_ack,drop_read,stuck_off,bad_goal,settle_required,drift_after_enable;
static const int low[6]={725,815,862,846,147,1975};
static const int high[6]={3458,3208,3078,3215,3996,3567};
static const int offset[6]={-1429,-1868,-551,1828,-773,1727};
static void word(unsigned char *p,int n){p[0]=(unsigned char)n;p[1]=(unsigned char)(n>>8);}
static uint32_t now(void *p){(void)p;return tick++;}
static int recv(void *p,uint8_t *ch){(void)p;if(at==len)return 0;*ch=reply[at++];return 1;}
static int send(void *ctx,const uint8_t *p,unsigned n)
{
    (void)ctx;unsigned sum=0;for(unsigned i=2;i<n;++i)sum+=p[i];
    assert((sum&255)==255&&p[2]>=1&&p[2]<=6);
    unsigned char *m=mem[p[2]-1];at=len=0;
    if(p[4]==3){
        /* Hardware contract: no EEPROM, broadcasts, or unknown RAM writes. */
        assert((p[5]==40&&n==8)||(p[5]==41&&n==14));writes++;
        if(p[5]==40&&p[6]){
            assert(m[42]==m[56]&&m[43]==m[57]);enables++;enable_tick=tick;
            if(drift_after_enable)word(m+56,2410);
            for(int i=0;i<6;++i)if(i!=(int)p[2]-1)assert(mem[i][40]==0);
        }
        if(p[5]==41&&m[40]){
            ++target_writes;
            if(settle_required)assert(tick-enable_tick>=150U);
        }
        if(!(stuck_off&&p[5]==40&&!p[6]))memcpy(m+p[5],p+6,n-7);
        if(bad_goal&&p[5]==41)m[42]^=1;
        if(no_write_ack)return (int)n;
    }
    if(p[4]==2&&drop_read==(int)p[5])return (int)n;
    unsigned count=p[4]==2?p[6]:0;
    reply[len++]=255;reply[len++]=255;reply[len++]=p[2];
    reply[len++]=(uint8_t)(count+2);reply[len++]=0;
    for(unsigned i=0;i<count;++i)reply[len++]=m[p[5]+i];
    sum=0;for(unsigned i=2;i<len;++i)sum+=reply[i];reply[len++]=(uint8_t)~sum;
    return (int)n;
}
static void reset(motor_controller *c,st_bus *b)
{
    memset(mem,0,sizeof mem);at=len=tick=writes=enables=0;no_write_ack=drop_read=stuck_off=bad_goal=settle_required=drift_after_enable=0;enable_tick=target_writes=0;
    for(int i=0;i<6;++i){
        word(mem[i]+9,low[i]);word(mem[i]+11,high[i]);
        int raw=offset[i]<0?(-offset[i]|2048):offset[i];word(mem[i]+31,raw);
        word(mem[i]+56,2400);mem[i][62]=121;mem[i][63]=32;
    }
    motor_init(c,b);
}
int main(void)
{
    st_bus b={0,send,recv,now};motor_controller c;reset(&c,&b);
    assert(motor_jog(&c,1,6,100)==ST_OK&&enables==1&&c.active);
    assert(motor_jog(&c,2,1,10)==MOTOR_BUSY);
    word(mem[5]+56,2498);
    for(int i=0;i<3;++i){tick+=80;motor_poll(&c);}
    assert(!c.active&&c.outcome==ST_OK&&c.off_confirmed&&mem[5][40]==0);
    unsigned before=writes;assert(motor_jog(&c,1,6,100)==MOTOR_STALE&&writes==before);
    reset(&c,&b);mem[1][40]=1;
    assert(motor_jog(&c,1,6,100)==MOTOR_TORQUE_ACTIVE&&writes==0);
    reset(&c,&b);mem[5][9]^=1;
    assert(motor_jog(&c,1,6,100)==MOTOR_CALIBRATION&&writes==0);
    reset(&c,&b);mem[5][31]^=1;
    assert(motor_jog(&c,1,6,100)==MOTOR_CALIBRATION&&writes==0);
    reset(&c,&b);mem[5][33]=1;
    assert(motor_jog(&c,1,6,100)==MOTOR_CALIBRATION&&writes==0);
    reset(&c,&b);word(mem[5]+56,high[5]-20);
    assert(motor_jog(&c,1,6,100)==MOTOR_LIMIT&&writes==0);
    reset(&c,&b);mem[5][62]=89;
    assert(motor_jog(&c,1,6,100)==MOTOR_HEALTH&&writes==0);
    reset(&c,&b);mem[5][63]=56;
    assert(motor_jog(&c,1,6,100)==MOTOR_HEALTH&&writes==0);
    reset(&c,&b);assert(motor_jog(&c,1,6,101)==MOTOR_ARGUMENT&&writes==0);
    reset(&c,&b);no_write_ack=1;
    assert(motor_jog(&c,1,6,-100)==ST_OK);motor_cancel(&c);
    assert(!c.active&&c.outcome==MOTOR_STOPPED&&c.off_confirmed);
    reset(&c,&b);assert(motor_jog(&c,1,6,100)==ST_OK);
    tick+=5001;motor_poll(&c);
    assert(!c.active&&c.outcome==MOTOR_STALLED&&c.off_confirmed);
    reset(&c,&b);assert(motor_jog(&c,1,6,100)==ST_OK);
    drop_read=56;tick+=80;motor_poll(&c);
    assert(!c.active&&c.outcome==ST_TIMEOUT&&c.off_confirmed);
    reset(&c,&b);assert(motor_jog(&c,1,6,100)==ST_OK);
    word(mem[5]+56,2520);tick+=80;motor_poll(&c);
    assert(!c.active&&c.outcome==MOTOR_HEALTH&&c.off_confirmed);
    reset(&c,&b);bad_goal=1;
    assert(motor_jog(&c,1,5,60)==ST_FORMAT&&enables==0&&c.off_confirmed);
    assert(strcmp(c.stage,"PREPARE_ORIGIN_GOAL")==0);
    st_trace_record trace;assert(st_trace_get(st_trace_count()-1,&trace)==ST_OK&&trace.op==2&&trace.address==42);
    reset(&c,&b);drop_read=42;
    assert(motor_jog(&c,1,6,100)==ST_TIMEOUT&&enables==0&&c.off_confirmed&&strcmp(c.stage,"PREPARE_ORIGIN_GOAL")==0);
    reset(&c,&b);assert(motor_jog(&c,1,6,100)==ST_OK);
    stuck_off=1;motor_cancel(&c);
    assert(!c.active&&!c.off_confirmed&&c.outcome==MOTOR_STOPPED);
    reset(&c,&b);assert(motor_jog(&c,1,5,-228)==ST_OK);
    assert(mem[4][46]==100&&mem[4][47]==0);
    word(mem[4]+56,2172);for(int i=0;i<3;i++){tick+=80;motor_poll(&c);}
    assert(!c.active&&c.outcome==ST_OK&&c.off_confirmed);
    reset(&c,&b);assert(motor_jog(&c,1,4,228)==MOTOR_ARGUMENT&&writes==0);
    reset(&c,&b);assert(motor_jog(&c,1,5,241)==MOTOR_ARGUMENT&&writes==0);
    reset(&c,&b);word(mem[4]+56,low[4]+100);
    assert(motor_jog(&c,1,5,-228)==MOTOR_LIMIT&&writes==0);
    reset(&c,&b);settle_required=1;
    assert(motor_jog(&c,1,5,-228)==ST_OK&&target_writes==1);
    motor_cancel(&c);assert(c.off_confirmed);
    reset(&c,&b);drift_after_enable=1;
    assert(motor_jog(&c,1,5,-228)==MOTOR_HEALTH&&target_writes==0&&c.off_confirmed);
    assert(strcmp(c.stage,"VERIFY_SETTLED_POSITION")==0);
    puts("PASS: bounded writes, current goal before enable, all-off preflight, calibration/range/health checks, sequence/busy, completion, stop, deadline, lost feedback, suppressed ack, failed goal readback, failed torque-off reporting");
    return 0;
}

