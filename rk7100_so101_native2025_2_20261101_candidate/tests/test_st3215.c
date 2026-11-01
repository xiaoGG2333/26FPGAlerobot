#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../firmware/st3215.h"
static uint8_t reply[100],last[16];
static unsigned len,at,clock_ms,lastlen;
static int mode,write_replies=1,torque=1;
static uint32_t now(void *c){(void)c;return clock_ms++;}
static int recv_byte(void *c,uint8_t *b){(void)c;if(at==len)return 0;*b=reply[at++];return 1;}
static int send_bytes(void *c,const uint8_t *p,unsigned count)
{
    (void)c;memcpy(last,p,count);lastlen=count;at=len=0;
    unsigned sum=0;for(unsigned i=2;i<count;++i)sum+=p[i];assert((sum&255)==255);
    assert(p[2]!=254);assert(p[4]==1||p[4]==2||(p[4]==3&&p[5]==40&&p[6]==0));
    if(mode==1)return (int)count;
    if(mode==8)return -1;
    if(p[4]==3){torque=0;if(!write_replies)return (int)count;}
    unsigned data_count=p[4]==2?p[6]:0;
    if(mode==7){memcpy(reply,p,count);len=count;}
    unsigned off=len;
    reply[len++]=255;reply[len++]=255;reply[len++]=(uint8_t)(mode==3?2:p[2]);
    reply[len++]=(uint8_t)(data_count+2);reply[len++]=(uint8_t)(mode==4?4:0);
    for(unsigned i=0;i<data_count;++i)reply[len++]=(p[5]==40)?(uint8_t)torque:(uint8_t)(i+1);
    sum=0;for(unsigned i=off+2;i<len;++i)sum+=reply[i];reply[len++]=(uint8_t)~sum;
    if(mode==2)reply[len-1]^=1;
    if(mode==5)reply[off+3]=255;
    if(mode==6)--len;
    return (int)count;
}
int main(void)
{
    st_bus bus={0,send_bytes,recv_byte,now};uint8_t err,data[32];
    assert(st_ping(&bus,1,&err)==ST_OK&&lastlen==6);
    assert(st_read(&bus,1,56,15,data,&err)==ST_OK&&data[0]==1&&data[14]==15);
    mode=1;assert(st_ping(&bus,1,&err)==ST_TIMEOUT);
    mode=2;assert(st_ping(&bus,1,&err)==ST_CHECKSUM);
    mode=3;assert(st_ping(&bus,1,&err)==ST_TIMEOUT);
    mode=4;assert(st_ping(&bus,1,&err)==ST_DEVICE_ERROR&&err==4);
    mode=5;assert(st_ping(&bus,1,&err)==ST_FORMAT);
    mode=6;assert(st_ping(&bus,1,&err)==ST_TIMEOUT);
    mode=7;assert(st_read(&bus,1,56,15,data,&err)==ST_OK);
    mode=8;assert(st_ping(&bus,1,&err)==ST_TX_ERROR);
    mode=0;assert(st_torque_off(&bus,1,&err)==ST_OK&&torque==0);
    torque=1;write_replies=0;assert(st_torque_off(&bus,1,&err)==ST_OK&&torque==0);
    assert(st_ping(&bus,254,&err)==ST_ARGUMENT);
    assert(st_read(&bus,1,250,15,data,&err)==ST_ARGUMENT);
    puts("PASS: packet framing, read, timeout, checksum, wrong ID, device error, malformed/truncated, echo, TX error, torque-off readback, suppressed WRITE reply, argument checks");
    return 0;
}
