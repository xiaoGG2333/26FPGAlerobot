/* Hardware-free tests for diagnostic capture only. No serial port is opened. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/st3215.h"

enum response_mode {
    NORMAL, ECHO, BAD_CHECKSUM, DEVICE_ERROR, WRONG_ID, WRONG_LENGTH,
    NO_RESPONSE, TX_ERROR, RX_OVERFLOW, STALE_THEN_FRESH, BAD_THEN_GOOD
};
typedef struct {
    uint8_t incoming[512],last_tx[16];
    unsigned length,at,ticks,sends,last_tx_length;
    enum response_mode mode;
} fixture;
static unsigned tests_run;

/* Each byte costs less than 1 ms, as on the real 1 Mbaud bus. */
static uint32_t milliseconds(void *context)
{fixture *f=context;return f->ticks++/32U;}
static int receive_byte(void *context,uint8_t *byte)
{
    fixture *f=context;
    if(f->at==f->length)return 0;
    *byte=f->incoming[f->at++];return 1;
}
static void append_packet(fixture *f,unsigned id,unsigned count,unsigned error,unsigned temperature)
{
    unsigned begin=f->length,sum=0;
    assert(f->length+count+6U<=sizeof f->incoming);
    f->incoming[f->length++]=255;f->incoming[f->length++]=255;
    f->incoming[f->length++]=(uint8_t)id;
    f->incoming[f->length++]=(uint8_t)(count+2U);
    f->incoming[f->length++]=(uint8_t)error;
    for(unsigned i=0;i<count;++i)f->incoming[f->length++]=(uint8_t)(i==0?121:temperature);
    for(unsigned i=begin+2;i<f->length;++i)sum+=f->incoming[i];
    f->incoming[f->length++]=(uint8_t)~sum;
}
static int send_bytes(void *context,const uint8_t *bytes,unsigned count)
{
    fixture *f=context;unsigned sum=0;
    assert(count<=sizeof f->last_tx);
    memcpy(f->last_tx,bytes,count);f->last_tx_length=count;++f->sends;
    for(unsigned i=2;i<count;++i)sum+=bytes[i];
    assert((sum&255U)==255U);
    /* Preserve pending input rather than silently dropping stale frames. */
    unsigned pending=f->length-f->at;
    memmove(f->incoming,f->incoming+f->at,pending);f->at=0;f->length=pending;
    if(f->mode==TX_ERROR)return -1;
    if(f->mode==NO_RESPONSE)return (int)count;
    unsigned data_count=bytes[4]==2?bytes[6]:0;
    if(f->mode==ECHO){memcpy(f->incoming+f->length,bytes,count);f->length+=count;}
    if(f->mode==RX_OVERFLOW){
        for(unsigned i=0;i<13;++i)append_packet(f,bytes[2]==1?2:1,data_count,0,31);
    }
    if(f->mode==STALE_THEN_FRESH){
        append_packet(f,bytes[2],data_count,0,73);
        append_packet(f,bytes[2],data_count,0,31);return (int)count;
    }
    if(f->mode==BAD_THEN_GOOD){
        append_packet(f,bytes[2],data_count,0,73);f->incoming[f->length-1]^=1;
    }
    append_packet(f,f->mode==WRONG_ID?(bytes[2]==1?2:1):bytes[2],
                  f->mode==WRONG_LENGTH?data_count+1:data_count,
                  f->mode==DEVICE_ERROR?4:0,31);
    if(f->mode==BAD_CHECKSUM)f->incoming[f->length-1]^=1;
    return (int)count;
}
static st_bus setup(fixture *f,enum response_mode mode)
{
    memset(f,0,sizeof *f);f->mode=mode;st_trace_clear();
    st_bus bus={f,send_bytes,receive_byte,milliseconds};return bus;
}
static st_trace_record record_at(unsigned index)
{
    st_trace_record record;assert(st_trace_get(index,&record)==ST_OK);return record;
}
static void test_normal_read(void)
{
    fixture f;st_bus bus=setup(&f,NORMAL);uint8_t data[2],error=255;
    assert(st_read(&bus,4,62,2,data,&error)==ST_OK);
    assert(data[0]==121&&data[1]==31&&error==0&&st_trace_count()==1);
    st_trace_record r=record_at(0);
    assert(r.id==4&&r.op==2&&r.address==62&&r.count==2&&r.rc==ST_OK);
    assert(r.tx_length==8&&r.rx_length==8&&r.drain==0&&!r.truncated);
    assert(!memcmp(r.tx,f.last_tx,r.tx_length)&&r.rx[5]==121&&r.rx[6]==31);
    assert(r.end_ms>=r.start_ms&&r.end_ms-r.start_ms<30);++tests_run;
}
static void test_echo_capture(void)
{
    fixture f;st_bus bus=setup(&f,ECHO);uint8_t data[2],error;
    assert(st_read(&bus,1,62,2,data,&error)==ST_OK&&data[1]==31);
    st_trace_record r=record_at(0);
    assert(r.rc==ST_OK&&r.rx_length==16&&!memcmp(r.rx,r.tx,8));
    assert(r.rx[14]==31);++tests_run;
}
static void test_exit_codes(void)
{
    static const enum response_mode modes[]={BAD_CHECKSUM,DEVICE_ERROR,WRONG_ID,WRONG_LENGTH,NO_RESPONSE,TX_ERROR};
    static const int expected[]={ST_CHECKSUM,ST_DEVICE_ERROR,ST_TIMEOUT,ST_FORMAT,ST_TIMEOUT,ST_TX_ERROR};
    for(unsigned i=0;i<sizeof modes/sizeof modes[0];++i){
        fixture f;st_bus bus=setup(&f,modes[i]);uint8_t data[2],error=255;
        assert(st_read(&bus,1,62,2,data,&error)==expected[i]);
        st_trace_record r=record_at(0);assert(r.rc==expected[i]&&r.tx_length==8);
        if(modes[i]==DEVICE_ERROR)assert(error==4);
        if(modes[i]==NO_RESPONSE||modes[i]==TX_ERROR)assert(r.rx_length==0);
        else assert(r.rx_length>0);
        ++tests_run;
    }
}
static void test_pre_drain(void)
{
    fixture f;st_bus bus=setup(&f,NORMAL);uint8_t data[2],error;
    memset(f.incoming,0x21,3);f.length=3;
    assert(st_read(&bus,1,62,2,data,&error)==ST_OK);
    st_trace_record r=record_at(0);
    assert(r.drain==3&&r.rx_length==8&&r.rx[0]==255);++tests_run;
}
static void test_bounded_pre_drain(void)
{
    fixture f;st_bus bus=setup(&f,NORMAL);uint8_t data[2],error;
    memset(f.incoming,0x21,140);f.length=140;
    assert(st_read(&bus,1,62,2,data,&error)==ST_OK);
    st_trace_record r=record_at(0);
    assert(r.drain==128&&r.rx_length==20&&r.rx[0]==0x21&&r.rx[12]==255);++tests_run;
}
static void test_truncation(void)
{
    fixture f;st_bus bus=setup(&f,RX_OVERFLOW);uint8_t data[2],error;
    assert(st_read(&bus,1,62,2,data,&error)==ST_OK&&data[1]==31);
    st_trace_record r=record_at(0);
    assert(r.rc==ST_OK&&r.rx_length==ST_TRACE_RX_MAX&&r.truncated==1);
    assert(f.at==112);++tests_run;
}
static void test_checksum_evidence_before_good_frame(void)
{
    fixture f;st_bus bus=setup(&f,BAD_THEN_GOOD);uint8_t data[2],error;
    assert(st_read(&bus,1,62,2,data,&error)==ST_OK&&data[1]==31);
    st_trace_record r=record_at(0);unsigned sum=0;
    assert(r.rx_length==16&&r.rc==ST_OK&&r.rx[6]==73&&r.rx[14]==31);
    for(unsigned i=2;i<8;++i)sum+=r.rx[i];
    assert((sum&255U)!=255U);++tests_run;
}
static void test_same_id_same_length_stale_frame(void)
{
    /* Preserve and demonstrate existing parser behavior; this test does not fix it. */
    fixture f;st_bus bus=setup(&f,STALE_THEN_FRESH);uint8_t data[2],error;
    assert(st_read(&bus,1,62,2,data,&error)==ST_OK&&data[1]==73);
    st_trace_record first=record_at(0);
    assert(first.rc==ST_OK&&first.rx_length==8&&first.rx[6]==73&&f.length-f.at==8);
    f.mode=NORMAL;
    assert(st_read(&bus,1,62,2,data,&error)==ST_OK&&data[1]==31);
    st_trace_record next=record_at(1);assert(next.drain==8&&next.rx[6]==31);++tests_run;
}
static void test_freeze_and_clear(void)
{
    fixture f;st_bus bus=setup(&f,NORMAL);uint8_t error;
    assert(st_ping(&bus,1,&error)==ST_OK);st_trace_record before=record_at(0);
    st_trace_freeze();st_trace_freeze();
    f.mode=BAD_CHECKSUM;assert(st_ping(&bus,2,&error)==ST_CHECKSUM);
    assert(st_trace_count()==1);st_trace_record after=record_at(0);
    assert(!memcmp(&before,&after,sizeof before)&&f.sends==2);
    st_trace_clear();assert(st_trace_count()==0);
    f.mode=NORMAL;assert(st_ping(&bus,3,&error)==ST_OK);
    assert(st_trace_count()==1&&record_at(0).id==3);++tests_run;
}
static void test_ring_wrap(void)
{
    fixture f;st_bus bus=setup(&f,NORMAL);uint8_t error;
    for(unsigned id=1;id<=40;++id)assert(st_ping(&bus,id,&error)==ST_OK);
    assert(st_trace_count()==ST_TRACE_CAPACITY);
    for(unsigned i=0;i<ST_TRACE_CAPACITY;++i)assert(record_at(i).id==i+9);
    st_trace_record record;
    assert(st_trace_get(ST_TRACE_CAPACITY,&record)==ST_ARGUMENT);
    assert(st_trace_get(0,0)==ST_ARGUMENT);++tests_run;
}
static void test_argument_exit(void)
{
    fixture f;st_bus bus=setup(&f,NORMAL);uint8_t error;
    assert(st_ping(&bus,254,&error)==ST_ARGUMENT);
    st_trace_record r=record_at(0);
    assert(r.rc==ST_ARGUMENT&&r.id==254&&r.op==1&&r.tx_length==0&&r.rx_length==0&&f.sends==0);
    assert(st_ping(0,1,&error)==ST_ARGUMENT);
    r=record_at(1);assert(r.rc==ST_ARGUMENT&&r.start_ms==0&&r.end_ms==0);++tests_run;
}
int main(void)
{
    test_normal_read();test_echo_capture();test_exit_codes();test_pre_drain();
    test_bounded_pre_drain();test_truncation();test_checksum_evidence_before_good_frame();
    test_same_id_same_length_stale_frame();test_freeze_and_clear();test_ring_wrap();test_argument_exit();
    printf("PASS: %u diagnostic cases; no serial or hardware I/O\n",tests_run);return 0;
}
