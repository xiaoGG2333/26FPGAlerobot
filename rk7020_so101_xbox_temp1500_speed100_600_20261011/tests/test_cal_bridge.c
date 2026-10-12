#include "../firmware/cal_bridge.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct {
    uint8_t memory[6][80],rx[64];unsigned used,length,writes,enables,goal_writes;
    uint32_t now;int missing_id,fail_read_once,fail_read_address,fail_read_id;
    unsigned clock_calls,after_write_addr,after_write_faults,after_write_reads;
    int after_write_kind,after_write_armed;uint8_t previous[2];
    int drift_after_goal;
    int auto_enable;unsigned auto_enables;
    unsigned log_count;uint8_t log_id[512],log_address[512];uint16_t log_value[512];
} fixture;
static int send_packet(void *ctx,const uint8_t *p,unsigned n)
{
    fixture *f=ctx;assert(n>=6&&p[0]==255&&p[1]==255&&p[2]>=1&&p[2]<=6);
    unsigned sum=0;for(unsigned i=2;i<n;++i)sum+=p[i];assert((sum&255)==255);
    unsigned id=p[2],address=n>6?p[5]:0,count=0;
    uint8_t err=0;
    if(p[4]==3){
        ++f->writes;count=n-7;
        if(address==f->after_write_addr&&f->after_write_faults){
            memcpy(f->previous,f->memory[id-1]+address,count);f->after_write_armed=1;
        }
        if(f->log_count<512){unsigned k=f->log_count++;f->log_id[k]=(uint8_t)id;f->log_address[k]=(uint8_t)address;
            f->log_value[k]=(uint16_t)(p[6]+(count==2?((unsigned)p[7]<<8):0));}
        if(address==40&&p[6])++f->enables;
        if(address==42)++f->goal_writes;
        if(address<40&&(f->memory[id-1][55]||f->memory[id-1][40]))err=1;
        else memcpy(f->memory[id-1]+address,p+6,count);
        if(f->auto_enable&&((address==42&&f->memory[id-1][33]==0)||
           (address==46&&f->memory[id-1][33]==1))){
            if(!f->memory[id-1][40])++f->auto_enables;
            f->memory[id-1][40]=1;
        }
        if(address==42&&f->drift_after_goal){
            int pos=f->memory[id-1][56]+((int)f->memory[id-1][57]<<8)+f->drift_after_goal;
            f->memory[id-1][56]=(uint8_t)pos;f->memory[id-1][57]=(uint8_t)(pos>>8);
        }
        count=0;
    }else {assert(p[4]==2);count=p[6];}
    f->used=0;f->length=0;
    if(f->missing_id==(int)id)return (int)n;
    if(p[4]==2&&f->fail_read_once&&address==(unsigned)f->fail_read_address&&
       (!f->fail_read_id||f->fail_read_id==(int)id)){
        --f->fail_read_once;return (int)n;
    }
    int post_fault=0;
    if(p[4]==2&&f->after_write_armed&&address==f->after_write_addr){
        ++f->after_write_reads;
        if(f->after_write_faults){--f->after_write_faults;post_fault=f->after_write_kind;}
    }
    if(post_fault==1)return (int)n;
    if(post_fault==5)return -1;
    if(post_fault==4)err=4;
    f->rx[0]=f->rx[1]=255;f->rx[2]=(uint8_t)id;f->rx[3]=(uint8_t)(count+2);f->rx[4]=err;
    if(count)memcpy(f->rx+5,f->memory[id-1]+address,count);
    if(post_fault==3&&count)memcpy(f->rx+5,f->previous,count);
    sum=0;for(unsigned i=2;i<count+5;++i)sum+=f->rx[i];f->rx[count+5]=(uint8_t)~sum;
    if(post_fault==2)f->rx[count+5]^=1;
    f->length=count+6;return (int)n;
}
static int receive_packet(void *ctx,uint8_t *p)
{fixture *f=ctx;if(f->used>=f->length){++f->now;return 0;}*p=f->rx[f->used++];return 1;}
static uint32_t clock_ms(void *ctx){fixture *f=ctx;return f->now+(f->clock_calls++/16U);}
static void set_word(uint8_t *p,unsigned x){p[0]=(uint8_t)x;p[1]=(uint8_t)(x>>8);}
static unsigned get_word(const uint8_t *p){return p[0]+((unsigned)p[1]<<8);}
static void init(fixture *f,st_bus *bus,cal_controller *c)
{
    memset(f,0,sizeof *f);
    for(int i=0;i<6;++i){
        uint8_t *m=f->memory[i];set_word(m+3,777);m[5]=(uint8_t)(i+1);
        set_word(m+9,500);set_word(m+11,3500);set_word(m+16,1000);set_word(m+31,100+i);
        m[33]=0;m[34]=50;m[35]=100;m[36]=80;m[40]=0;m[41]=5;
        set_word(m+42,2000);set_word(m+48,300);m[55]=1;set_word(m+56,2000);m[62]=122;m[63]=31;
    }
    *bus=(st_bus){f,send_packet,receive_packet,clock_ms};cal_init(c,bus);
}
static int w(cal_controller *c,unsigned seq,unsigned id,unsigned address,unsigned size,unsigned value)
{uint8_t p[2];set_word(p,value);return cal_write(c,seq,id,address,size,p);}
static void velocity(fixture *f,cal_controller *c,unsigned *seq,unsigned id)
{
    (void)f;
    assert(w(c,++*seq,id,55,1,0)==0);assert(w(c,++*seq,id,33,1,1)==0);
    assert(w(c,++*seq,id,16,2,600)==0);
    assert(w(c,++*seq,id,46,2,0)==0);assert(w(c,++*seq,id,40,1,1)==0);
}
int main(void)
{
    fixture f;st_bus b;cal_controller c;uint8_t out[32];unsigned seq=0;
    init(&f,&b,&c);assert(cal_begin(&c)==0&&c.active&&c.snapshot_valid);
    assert(f.writes==0&&f.enables==0);assert(cal_read(&c,1,1,0,32,out)==0);
    assert(cal_read(&c,1,1,0,1,out)==CAL_STALE);
    assert(cal_read(&c,2,7,0,1,out)==CAL_FORBIDDEN);
    assert(cal_read(&c,3,1,79,2,out)==CAL_FORBIDDEN);
    assert(w(&c,4,1,6,1,0)==CAL_FORBIDDEN);
    assert(w(&c,5,1,16,2,601)==CAL_FORBIDDEN);
    assert(w(&c,6,1,31,2,4096)==CAL_FORBIDDEN);
    assert(w(&c,7,1,46,2,401)==CAL_FORBIDDEN);
    assert(w(&c,8,1,55,1,2)==CAL_FORBIDDEN&&f.writes==0);
    set_word(f.memory[0]+42,1000);assert(w(&c,9,1,40,1,1)==CAL_ENABLE_GUARD&&f.enables==0);
    set_word(f.memory[0]+42,2000);set_word(f.memory[0]+48,1000);
    assert(w(&c,10,1,40,1,1)==CAL_ENABLE_GUARD&&f.enables==0);
    set_word(f.memory[0]+48,300);set_word(f.memory[0]+16,600);
    assert(w(&c,11,1,40,1,1)==CAL_ENABLE_GUARD&&f.enables==0);
    assert(w(&c,12,1,42,2,2000)==0);assert(w(&c,13,1,40,1,1)==0&&f.enables==1);
    assert(w(&c,14,1,33,1,1)==CAL_TORQUE_ACTIVE);assert(w(&c,15,1,40,1,0)==0);
    assert(cal_finish(&c,0)==0&&c.off_confirmed&&c.rollback_success);
    puts("BEGIN/read/whitelist/sequence/enable guards PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);seq=0;
    velocity(&f,&c,&seq,1);assert(w(&c,++seq,1,46,2,300)==0);
    assert(c.moving_id==1);
    velocity(&f,&c,&seq,2);assert(w(&c,++seq,2,46,2,32768+300)==CAL_SECOND_VELOCITY);
    assert(get_word(f.memory[1]+46)==0);
    assert(w(&c,++seq,1,40,1,0)==0);
    assert(w(&c,++seq,1,40,1,1)==CAL_ENABLE_GUARD);
    f.now=c.last_rpc+CAL_WATCHDOG_MS+1;cal_poll(&c);
    assert(!c.active&&c.event_pending&&c.off_confirmed&&c.rollback_success);
    for(int i=0;i<6;++i){assert(f.memory[i][40]==0&&f.memory[i][33]==0&&f.memory[i][55]==1);assert(get_word(f.memory[i]+46)==0);}
    puts("single velocity axis/watchdog rollback PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);seq=0;velocity(&f,&c,&seq,6);
    f.after_write_addr=46;f.after_write_faults=3;f.after_write_kind=1;f.log_count=0;
    assert(w(&c,++seq,6,46,2,300)==ST_TIMEOUT);
    assert(f.log_count>=3&&f.log_id[0]==6&&f.log_address[0]==46&&f.log_value[0]==300);
    assert(f.log_id[1]==6&&f.log_address[1]==40&&f.log_value[1]==0);
    assert(f.log_id[2]==1&&f.log_address[2]==40&&f.log_value[2]==0);
    for(unsigned k=1;k<=6;++k)assert(f.log_address[k]==40&&f.log_value[k]==0);
    assert(!c.active&&c.off_confirmed&&c.rollback_success&&!c.moving_id);
    unsigned nonzero=0;for(unsigned k=0;k<f.log_count;++k)if(f.log_address[k]==46&&f.log_value[k])++nonzero;
    assert(nonzero==1);
    puts("uncertain nonzero velocity stops the possible axis first, without retry PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);seq=0;velocity(&f,&c,&seq,1);
    f.memory[0][41]=11;f.log_count=0;
    assert(w(&c,++seq,1,46,2,300)==CAL_ENABLE_GUARD);
    assert(!c.active&&c.off_confirmed&&c.rollback_success);
    for(unsigned k=0;k<f.log_count;++k)assert(f.log_address[k]!=46||f.log_value[k]==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);seq=0;velocity(&f,&c,&seq,1);
    f.memory[0][41]=10;assert(w(&c,++seq,1,46,2,300)==0&&c.active&&c.moving_id==1);
    assert(cal_finish(&c,0)==0&&c.off_confirmed&&c.rollback_success);
    puts("nonzero velocity validates ACC before WRITE, rejects above 10 without motion, accepts limit 10 PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);seq=0;
    assert(w(&c,++seq,1,55,1,0)==0);assert(w(&c,++seq,1,31,2,0)==0);
    assert(w(&c,++seq,1,9,2,0)==0);assert(w(&c,++seq,1,11,2,4095)==0);
    assert(w(&c,++seq,1,16,2,400)==0);assert(w(&c,++seq,1,34,1,20)==0);
    assert(cal_finish(&c,0)==0);
    assert(get_word(f.memory[0]+31)==100&&get_word(f.memory[0]+9)==500&&get_word(f.memory[0]+11)==3500);
    assert(get_word(f.memory[0]+16)==1000&&f.memory[0][34]==50&&f.memory[0][55]==1);
    assert(cal_begin(&c)==0);seq=0;
    assert(w(&c,++seq,1,55,1,0)==0);assert(w(&c,++seq,1,31,2,0)==0);
    assert(w(&c,++seq,1,9,2,0)==0);assert(w(&c,++seq,1,11,2,4095)==0);
    assert(w(&c,++seq,1,16,2,400)==0);assert(w(&c,++seq,1,34,1,20)==0);
    assert(cal_finish(&c,1)==0);
    assert(get_word(f.memory[0]+31)==0&&get_word(f.memory[0]+9)==0&&get_word(f.memory[0]+11)==4095);
    assert(get_word(f.memory[0]+16)==1000&&f.memory[0][34]==50&&f.memory[0][55]==1&&f.memory[0][40]==0);
    assert(!c.snapshot_valid&&cal_finish(&c,0)==CAL_INACTIVE&&get_word(f.memory[0]+31)==0);
    puts("STOP rollback/COMMIT preserve only offset and limits PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);seq=0;
    set_word(f.memory[0]+16,600);
    f.fail_read_once=3;f.fail_read_address=42;
    assert(w(&c,++seq,1,42,2,2003)==ST_TIMEOUT);
    assert(!c.active&&!c.fault&&c.off_confirmed&&c.rollback_success&&f.goal_writes==1);
    assert(w(&c,++seq,1,40,1,1)==CAL_INACTIVE);
    puts("uncertain write performs no retry and stops session PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);f.missing_id=6;
    assert(cal_finish(&c,0)==CAL_CLEANUP);
    assert(!c.active&&c.fault&&!c.off_confirmed&&!c.rollback_success);
    assert(c.first_fail_id==6&&c.first_fail_addr==40&&c.first_fail_rc==ST_TIMEOUT);
    assert(cal_begin(&c)==CAL_FAULT);
    f.missing_id=0;assert(cal_finish(&c,0)==0&&!c.fault&&c.off_confirmed);
    puts("failed OFF is explicit and blocks BEGIN until cleanup succeeds PASS");
    init(&f,&b,&c);f.memory[2][40]=1;assert(cal_begin(&c)==CAL_TORQUE_ACTIVE&&f.writes==0);
    init(&f,&b,&c);f.memory[2][62]=130;assert(cal_begin(&c)==CAL_HEALTH&&f.writes==0);
    init(&f,&b,&c);set_word(f.memory[2]+3,888);assert(cal_begin(&c)==CAL_FORBIDDEN&&f.writes==0);
    init(&f,&b,&c);f.memory[2][5]=4;assert(cal_begin(&c)==CAL_FORBIDDEN&&f.writes==0);
    init(&f,&b,&c);set_word(f.memory[2]+9,3500);assert(cal_begin(&c)==CAL_FORBIDDEN&&f.writes==0);
    init(&f,&b,&c);f.memory[2][33]=1;assert(cal_begin(&c)==CAL_FORBIDDEN&&f.writes==0);
    puts("preflight is read-only and rejects active/unhealthy arm PASS");
    init(&f,&b,&c);assert(cal_begin(&c)==0);seq=0;
    assert(w(&c,++seq,1,55,1,0)==0);
    unsigned before=f.writes;assert(w(&c,++seq,1,31,2,100)==0&&f.writes==before);
    f.after_write_addr=31;f.after_write_faults=1;f.after_write_kind=1;
    assert(w(&c,++seq,1,31,2,777)==0&&f.writes==before+1&&f.after_write_reads==2);
    assert(c.active&&get_word(f.memory[0]+31)==777);
    before=f.writes;f.after_write_faults=2;f.after_write_kind=3;f.after_write_reads=0;
    assert(w(&c,++seq,1,31,2,888)==0&&f.writes==before+1&&f.after_write_reads==3);
    assert(c.active&&get_word(f.memory[0]+31)==888);
    assert(cal_finish(&c,0)==0&&c.off_confirmed&&c.rollback_success);
    puts("EEPROM same-value skip; transient read timeout and old value resolve with exactly one WRITE PASS");
    for(int kind=1;kind<=5;++kind){
        init(&f,&b,&c);f.memory[0][55]=0;
        f.after_write_addr=31;f.after_write_faults=1;f.after_write_kind=kind;
        uint8_t value[2];set_word(value,777);uint8_t err=0;
        int rc=st_cal_write(&b,1,31,2,value,&err);
        assert(f.writes==1);
        if(kind<=3){assert(rc==ST_OK&&f.after_write_reads==2);}
        else {assert(rc==(kind==4?ST_DEVICE_ERROR:ST_TX_ERROR)&&f.after_write_reads==1);}
    }
    puts("READ-only recovery of timeout/checksum/old data; device/TX errors stop without READ or WRITE retry PASS");
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+16,600);
    set_word(f.memory[0]+42,1000);set_word(f.memory[0]+56,2010);f.drift_after_goal=3;
    assert(cal_hold(&c,1,1)==0&&c.active&&c.goal_ready[0]&&f.memory[0][40]==1);
    assert(get_word(f.memory[0]+42)==2010&&f.goal_writes==1&&f.enables==1&&!c.off_confirmed);
    assert(cal_finish(&c,0)==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+16,600);f.drift_after_goal=6;
    assert(cal_hold(&c,1,1)==CAL_ENABLE_GUARD&&!c.active&&c.off_confirmed&&f.enables==0&&f.goal_writes==1);
    init(&f,&b,&c);assert(cal_begin(&c)==0);f.memory[0][33]=1;
    assert(cal_hold(&c,1,1)==CAL_ENABLE_GUARD&&!c.active&&c.off_confirmed&&f.enables==0&&f.goal_writes==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);f.memory[0][40]=1;
    assert(cal_hold(&c,1,1)==CAL_TORQUE_ACTIVE&&!c.active&&c.off_confirmed&&f.enables==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+56,499);
    assert(cal_hold(&c,1,1)==CAL_ENABLE_GUARD&&!c.active&&c.off_confirmed&&f.enables==0&&f.goal_writes==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+46,1);
    assert(cal_hold(&c,1,1)==CAL_ENABLE_GUARD&&!c.active&&c.off_confirmed&&f.enables==0&&f.goal_writes==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);
    assert(cal_hold(&c,1,1)==CAL_ENABLE_GUARD&&!c.active&&c.off_confirmed&&f.enables==0);
    puts("atomic current HOLD; unchanged 5-count drift guard; mode/torque/speed/range/cap rejection and rollback PASS");
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+16,600);f.auto_enable=1;
    assert(cal_hold(&c,1,1)==0&&c.active&&f.memory[0][40]==1&&!c.off_confirmed);
    assert(f.goal_writes==1&&f.auto_enables==1&&f.enables==0);
    assert(cal_finish(&c,0)==0&&c.off_confirmed);
    init(&f,&b,&c);assert(cal_begin(&c)==0);f.auto_enable=1;
    assert(cal_hold(&c,1,1)==CAL_ENABLE_GUARD&&!c.active&&f.goal_writes==0&&f.auto_enables==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);f.auto_enable=1;
    assert(w(&c,1,1,42,2,2000)==CAL_ENABLE_GUARD&&!c.active&&f.goal_writes==0&&f.auto_enables==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+16,600);f.auto_enable=1;
    assert(w(&c,1,1,42,2,2006)==CAL_ENABLE_GUARD&&!c.active&&f.goal_writes==0&&f.auto_enables==0);
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+16,600);f.auto_enable=1;
    assert(w(&c,1,1,42,2,2003)==0&&f.auto_enables==1&&!c.off_confirmed);
    assert(cal_finish(&c,0)==0&&c.off_confirmed);
    init(&f,&b,&c);assert(cal_begin(&c)==0);set_word(f.memory[0]+16,600);f.memory[0][33]=1;f.auto_enable=1;
    assert(w(&c,1,1,46,2,0)==0&&f.auto_enables==1&&f.memory[0][40]==1&&!c.off_confirmed);
    assert(cal_finish(&c,0)==0&&c.off_confirmed&&f.memory[0][40]==0);
    puts("real-servo automatic torque on Goal42/velocity-zero; all goal guards precede motion; HOLD avoids duplicate ON PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);f.auto_enable=1;c.moving_id=6;f.log_count=0;
    for(unsigned id=1;id<=6;++id){f.memory[id-1][33]=1;f.memory[id-1][40]=1;set_word(f.memory[id-1]+46,300);}
    assert(cal_finish(&c,0)==0&&c.off_confirmed&&c.rollback_success&&!c.moving_id);
    assert(f.auto_enables==0&&f.enables==0);
    assert(f.log_id[0]==6&&f.log_address[0]==40&&f.log_value[0]==0);
    for(unsigned k=0;k<6;++k)assert(f.log_address[k]==40&&f.log_value[k]==0);
    for(unsigned id=1;id<=6;++id){
        assert(f.memory[id-1][40]==0&&f.memory[id-1][33]==0&&get_word(f.memory[id-1]+46)==0);
        unsigned mode_index=512,speed_index=512;
        for(unsigned k=0;k<f.log_count;++k)if(f.log_id[k]==id){
            if(f.log_address[k]==33)mode_index=k;
            if(f.log_address[k]==46)speed_index=k;
            assert(f.log_address[k]!=18);
        }
        assert(mode_index>=6&&mode_index<speed_index);
    }
    puts("STOP confirms all OFF first, then Mode0 before Speed0; Mode1 automatic-enable never occurs PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);f.auto_enable=1;c.moving_id=6;f.missing_id=6;f.log_count=0;
    f.memory[5][33]=1;f.memory[5][40]=1;set_word(f.memory[5]+46,300);
    assert(cal_finish(&c,0)==CAL_CLEANUP&&c.fault&&!c.off_confirmed&&!c.rollback_success&&c.moving_id==6);
    assert(c.first_fail_id==6&&c.first_fail_addr==40&&c.first_fail_rc==ST_TIMEOUT);
    unsigned off_attempts=0;
    for(unsigned k=0;k<f.log_count;++k)if(f.log_id[k]==6){
        assert(f.log_address[k]==40&&f.log_value[k]==0);++off_attempts;
    }
    assert(off_attempts==3&&f.auto_enables==0&&f.memory[5][33]==1&&get_word(f.memory[5]+46)==300);
    assert(f.log_id[0]==6&&f.log_id[1]==1&&f.log_address[0]==40&&f.log_address[1]==40);
    f.missing_id=0;assert(cal_finish(&c,0)==0&&!c.fault&&!c.moving_id&&c.off_confirmed);
    assert(f.auto_enables==0&&f.memory[5][33]==0&&get_word(f.memory[5]+46)==0);
    puts("unconfirmed OFF forbids all Mode/Speed/EEPROM writes, retries are distributed, uncertain axis survives fault PASS");

    init(&f,&b,&c);assert(cal_begin(&c)==0);f.auto_enable=1;c.moving_id=6;f.log_count=0;
    f.memory[5][33]=1;f.memory[5][40]=1;set_word(f.memory[5]+46,300);
    f.fail_read_once=1;f.fail_read_address=33;f.fail_read_id=6;
    assert(cal_finish(&c,0)==CAL_CLEANUP&&c.fault&&c.off_confirmed&&!c.rollback_success&&c.moving_id==6);
    assert(c.first_fail_id==6&&c.first_fail_addr==33&&c.first_fail_rc==ST_TIMEOUT);
    for(unsigned k=0;k<f.log_count;++k)if(f.log_id[k]==6)assert(f.log_address[k]==40||f.log_address[k]==55);
    assert(f.auto_enables==0&&f.memory[5][33]==1&&get_word(f.memory[5]+46)==300&&f.memory[5][55]==1);
    assert(cal_finish(&c,0)==0&&!c.fault&&!c.moving_id&&f.memory[5][33]==0&&get_word(f.memory[5]+46)==0);
    puts("failed Mode0 readback forbids Speed0 and EEPROM restore; relock/final OFF verification remain safe PASS");
    return 0;
}
