/* ST/STS little-endian protocol. Calibration EEPROM writes use a dedicated
   register whitelist and one WRITE plus verified readback. Only arm Sync Write
   uses broadcast; calibration never broadcasts or changes ID/baud. */
#include "st3215.h"
#include <string.h>
static st_trace_record trace_records[ST_TRACE_CAPACITY];
static unsigned trace_next,trace_used;
static int trace_frozen;
void st_trace_clear(void)
{memset(trace_records,0,sizeof trace_records);trace_next=trace_used=0;trace_frozen=0;}
void st_trace_freeze(void){trace_frozen=1;}
unsigned st_trace_count(void){return trace_used;}
int st_trace_get(unsigned index,st_trace_record *record)
{
    if(!record||index>=trace_used)return ST_ARGUMENT;
    unsigned oldest=(trace_next+ST_TRACE_CAPACITY-trace_used)%ST_TRACE_CAPACITY;
    *record=trace_records[(oldest+index)%ST_TRACE_CAPACITY];return ST_OK;
}
static int trace_finish(st_trace_record *record,st_bus *bus,int rc)
{
    record->rc=rc;
    if(bus&&bus->milliseconds)record->end_ms=bus->milliseconds(bus->context);
    if(!trace_frozen){
        trace_records[trace_next]=*record;
        trace_next=(trace_next+1U)%ST_TRACE_CAPACITY;
        if(trace_used<ST_TRACE_CAPACITY)++trace_used;
    }
    return rc;
}
static int exchange(st_bus *bus,unsigned id,unsigned op,const uint8_t *args,
                    unsigned argc,unsigned expected,uint8_t *data,uint8_t *error)
{
    uint8_t tx[16],rx[40],ch;unsigned used=0,total=0,sum=0;
    st_trace_record trace;memset(&trace,0,sizeof trace);trace.id=id;trace.op=op;
    if(args&&argc<=8&&argc){trace.address=args[0];trace.count=op==2&&argc>=2?args[1]:argc-1;}
    if(bus&&bus->milliseconds)trace.start_ms=bus->milliseconds(bus->context);
    if(!bus||!bus->send||!bus->receive||!bus->milliseconds||!error||
       id<1||id>253||argc>8||expected>32||(!data&&expected))return trace_finish(&trace,bus,ST_ARGUMENT);
    *error=0;
    /* Bounded drain even for continuous noise. */
    for(unsigned i=0;i<128&&bus->receive(bus->context,&ch)>0;++i){++trace.drain;}
    tx[0]=255;tx[1]=255;tx[2]=(uint8_t)id;tx[3]=(uint8_t)(argc+2);tx[4]=(uint8_t)op;
    if(argc)memcpy(tx+5,args,argc);
    for(unsigned i=2;i<argc+5;++i)sum+=tx[i];
    tx[argc+5]=(uint8_t)~sum;
    trace.tx_length=argc+6;memcpy(trace.tx,tx,trace.tx_length);
    if(bus->send(bus->context,tx,argc+6)!=(int)(argc+6))return trace_finish(&trace,bus,ST_TX_ERROR);
    uint32_t start=bus->milliseconds(bus->context);int diagnostic=ST_TIMEOUT;
    while((uint32_t)(bus->milliseconds(bus->context)-start)<30U){
        if(bus->receive(bus->context,&ch)<=0)continue;
        if(trace.rx_length<ST_TRACE_RX_MAX)trace.rx[trace.rx_length++]=ch;
        else trace.truncated=1;
        if(used<2){if(ch==255)rx[used++]=ch;else used=0;continue;}
        if(used==2&&ch==255)continue;
        rx[used++]=ch;
        if(used==4){
            total=(unsigned)rx[3]+4;
            if(rx[3]<2||total>sizeof rx){used=0;diagnostic=ST_FORMAT;continue;}
        }
        if(total&&used==total){
            sum=0;for(unsigned i=2;i<total;++i)sum+=rx[i];
            if((sum&255U)!=255U){diagnostic=ST_CHECKSUM;}
            else if(rx[2]==id){
                /* Ignore an exact local echo of our instruction packet. */
                if(total==argc+6&&!memcmp(rx,tx,total)){}
                else if(rx[4]){*error=rx[4];return trace_finish(&trace,bus,ST_DEVICE_ERROR);}
                else if(rx[3]!=expected+2){diagnostic=ST_FORMAT;}
                else {if(expected)memcpy(data,rx+5,expected);return trace_finish(&trace,bus,ST_OK);}
            }
            used=0;total=0;
        }
    }
    return trace_finish(&trace,bus,diagnostic);
}
int st_ping(st_bus *b,unsigned id,uint8_t *err)
{return exchange(b,id,1,0,0,0,0,err);}
int st_read(st_bus *b,unsigned id,unsigned address,unsigned count,uint8_t *data,uint8_t *err)
{
    if(address>255||count<1||count>32||address+count>256)return ST_ARGUMENT;
    uint8_t args[2]={(uint8_t)address,(uint8_t)count};
    return exchange(b,id,2,args,2,count,data,err);
}
static int pause_ms(st_bus *b,uint32_t delay)
{
    uint32_t start=b->milliseconds(b->context);unsigned polls=0;
    while((uint32_t)(b->milliseconds(b->context)-start)<delay){
        /* Also bound a broken/non-advancing clock; do not spin forever. */
        if(++polls>=3000000U)return ST_TIMEOUT;
    }
    return ST_OK;
}
int st_cal_write(st_bus *b,unsigned id,unsigned address,unsigned count,const uint8_t *data,uint8_t *err)
{
    unsigned n=0;
    switch(address){
    case 9:case 11:case 16:case 31:case 42:case 46:case 48:n=2;break;
    case 33:case 34:case 35:case 36:case 40:case 41:case 55:n=1;break;
    default:return ST_ARGUMENT;
    }
    if(id<1||id>6||count!=n||!data)return ST_ARGUMENT;
    uint8_t args[3]={(uint8_t)address,0,0},actual[2];memcpy(args+1,data,count);
    if(address<40){
        int before=st_read(b,id,address,count,actual,err);
        if(before!=ST_OK)return before;
        if(!memcmp(actual,data,count))return ST_OK;
    }
    int rc=exchange(b,id,3,args,count+1,0,0,err);
    if(rc!=ST_OK&&rc!=ST_TIMEOUT&&rc!=ST_CHECKSUM&&rc!=ST_FORMAT)return rc;
    if(address<40){int delay_rc=pause_ms(b,10);if(delay_rc)return delay_rc;}
    /* EEPROM can briefly return the old value. Repeat only READ, never WRITE. */
    for(unsigned attempt=0;attempt<3;++attempt){
        rc=st_read(b,id,address,count,actual,err);
        if(rc==ST_OK){if(!memcmp(actual,data,count))return ST_OK;rc=ST_FORMAT;}
        else if(rc!=ST_TIMEOUT&&rc!=ST_CHECKSUM&&rc!=ST_FORMAT)return rc;
        if(attempt<2){int delay_rc=pause_ms(b,2);if(delay_rc)return delay_rc;}
    }
    return rc;
}
int st_torque_off(st_bus *b,unsigned id,uint8_t *err)
{
    uint8_t args[2]={40,0},readback=255;
    int rc=exchange(b,id,3,args,2,0,0,err);
    /* Some servos suppress WRITE replies; successful readback is authoritative. */
    if(rc!=ST_OK&&rc!=ST_TIMEOUT)return rc;
    rc=st_read(b,id,40,1,&readback,err);
    if(rc==ST_OK&&readback!=0)return ST_FORMAT;
    return rc;
}
int st_torque_on(st_bus *b,unsigned id,uint8_t *err)
{
    uint8_t args[2]={40,1},readback=0;
    int rc=exchange(b,id,3,args,2,0,0,err);
    if(rc!=ST_OK&&rc!=ST_TIMEOUT)return rc;
    rc=st_read(b,id,40,1,&readback,err);
    return rc==ST_OK&&readback!=1?ST_FORMAT:rc;
}
int st_set_goal(st_bus *b,unsigned id,unsigned pos,unsigned speed,unsigned acc,uint8_t *err)
{
    if(pos>4095||speed>100||acc>10)return ST_ARGUMENT;
    uint8_t args[8]={41,(uint8_t)acc,(uint8_t)pos,(uint8_t)(pos>>8),0,0,(uint8_t)speed,(uint8_t)(speed>>8)};
    int rc=exchange(b,id,3,args,8,0,0,err);
    if(rc!=ST_OK&&rc!=ST_TIMEOUT)return rc;
    uint8_t goal[2];rc=st_read(b,id,42,2,goal,err);
    if(rc==ST_OK && (unsigned)(goal[0]+((unsigned)goal[1]<<8))!=pos)return ST_FORMAT;
    return rc;
}
int st_sync_arm_goals(st_bus *b,const int pos[5],unsigned speed,unsigned acc,uint8_t *err)
{
    if(speed<1||speed>600)return ST_ARGUMENT;
    unsigned speeds[5]={speed,speed,speed,speed,speed};
    return st_sync_arm_goals_speeds(b,pos,speeds,acc,err);
}
int st_sync_arm_goals_speeds(st_bus *b,const int pos[5],const unsigned speeds[5],unsigned acc,uint8_t *err)
{
    if(!b||!b->send||!b->receive||!b->milliseconds||!pos||!speeds||!err||acc>5)return ST_ARGUMENT;
    for(int i=0;i<5;++i)if(pos[i]<0||pos[i]>4095||speeds[i]<1||speeds[i]>600)return ST_ARGUMENT;
    uint8_t tx[48]={255,255,254,44,0x83,41,7},ch;unsigned sum=0;
    for(int i=0;i<5;++i){
        unsigned o=7U+8U*(unsigned)i;
        tx[o]=(uint8_t)(i+1);tx[o+1]=(uint8_t)acc;
        tx[o+2]=(uint8_t)pos[i];tx[o+3]=(uint8_t)(pos[i]>>8);
        tx[o+4]=tx[o+5]=0;tx[o+6]=(uint8_t)speeds[i];tx[o+7]=(uint8_t)(speeds[i]>>8);
    }
    for(unsigned i=2;i<47;++i)sum+=tx[i];
    tx[47]=(uint8_t)~sum;
    for(unsigned i=0;i<128&&b->receive(b->context,&ch)>0;++i){}
    if(b->send(b->context,tx,sizeof tx)!=(int)sizeof tx)return ST_TX_ERROR;
    for(int i=0;i<5;++i){
        uint8_t p[7];int rc=st_read(b,(unsigned)(i+1),41,7,p,err);
        if(rc)return rc;
        if(p[0]!=acc||(int)(p[1]+((unsigned)p[2]<<8))!=pos[i]||p[3]||p[4]||
           (unsigned)(p[5]+((unsigned)p[6]<<8))!=speeds[i])return ST_FORMAT;
    }
    return ST_OK;
}
