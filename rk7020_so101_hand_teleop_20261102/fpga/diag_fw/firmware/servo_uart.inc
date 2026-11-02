#include "st3215.h"
#include "servo_mapping.h"
#include "xiltimer.h"
static XUartPs servo_uart;
static int servo_ready=0;
static unsigned servo_baud=1000000U;
static uint32_t servo_ms(void *unused)
{XTime now;(void)unused;XTime_GetTime(&now);return (uint32_t)(now/(COUNTS_PER_SECOND/1000U));}
static int servo_send(void *unused,const uint8_t *bytes,unsigned count)
{
    (void)unused;unsigned sent=0;uint32_t start=servo_ms(0);
    while(sent<count){
        sent+=XUartPs_Send(&servo_uart,(u8 *)(bytes+sent),count-sent);
        if((uint32_t)(servo_ms(0)-start)>30)return -1;
    }
    while(!XUartPs_IsTransmitEmpty(&servo_uart))
        if((uint32_t)(servo_ms(0)-start)>30)return -1;
    return (int)sent;
}
static int servo_receive(void *unused,uint8_t *byte)
{(void)unused;return (int)XUartPs_Recv(&servo_uart,byte,1);}
static st_bus servo_bus={0,servo_send,servo_receive,servo_ms};
static void servo_init(void)
{
    /* xiltimer starts the free-running global counter on the first sleep.
       Start it explicitly so every later timeout works even with queued input. */
    usleep(1);
    /* UART1 shares the configured 100MHz PS UART reference with UART0.
       The legacy BSP already contains the UART driver. Explicit configuration
       avoids an absent UART1 entry in its generated lookup table. Matching NEW
       ps7_init.c in this release's FSBL enables UART1 clock/reset and EMIO. */
    XUartPs_Config cfg=uart.Config;
    cfg.BaseAddress=0xE0001000U;cfg.ModemPinsConnected=0;
#ifdef SDT
    cfg.Name="uart1-emio";cfg.IntrId=0x4032U;cfg.RefClk=0x18U;
#else
    cfg.DeviceId=1;
#endif
    if(cfg.InputClockHz!=100000000U){send_text("SERVO1 INIT_ERROR UART_CLOCK\r\n");return;}
    if(XUartPs_CfgInitialize(&servo_uart,&cfg,cfg.BaseAddress)!=XST_SUCCESS ||
       XUartPs_SetBaudRate(&servo_uart,servo_baud)!=XST_SUCCESS){
        send_text("SERVO1 INIT_ERROR\r\n");return;
    }
    XUartPs_SetOperMode(&servo_uart,XUARTPS_OPER_MODE_NORMAL);
    XUartPs_SetOptions(&servo_uart,0U);servo_ready=1;
    send_text("SERVO1 READY UART1 EMIO J1.3=TX J1.4=RX baud=1000000 NO_AUTO_MOTION\r\n");
}
static const char *servo_reason(int rc)
{
    static const char *const names[]={"OK","TIMEOUT","CHECKSUM","FORMAT","TX_ERROR","DEVICE_ERROR","ARGUMENT"};
    return rc>=0&&rc<=6?names[rc]:"ERROR";
}
static void servo_report(unsigned id,int rc,uint8_t err)
{
    char line[120];snprintf(line,sizeof line,"SERVO1 ID=%u %s error=0x%02x\r\n",id,servo_reason(rc),err);send_text(line);
}
static int servo_word(const uint8_t *p,unsigned signbit)
{unsigned raw=p[0]+((unsigned)p[1]<<8);return raw&(1U<<signbit)?-(int)(raw&~(1U<<signbit)):(int)raw;}
static void servo_angles(void)
{
    int counts[5];uint8_t bytes[2],err=0;
    for(int i=0;i<5;++i){
        int rc=st_read(&servo_bus,(unsigned)so101_right_joints[i].id,56,2,bytes,&err);
        if(rc!=ST_OK){servo_report((unsigned)(i+1),rc,err);return;}
        counts[i]=servo_word(bytes,15);
    }
    float q[5];int rc=so101_counts_to_q(counts,q);
    if(rc){char line[80];snprintf(line,sizeof line,"SERVO1 ANGLES ERROR joint=%d ENCODER_RANGE\r\n",rc);send_text(line);return;}
    for(int i=0;i<5;++i){
        const so101_joint_map *j=&so101_right_joints[i];char line[180];
        snprintf(line,sizeof line,"SERVO1 ID=%d raw_pos=%d model_rad=%.9g model_deg=%.6f in_model=%d ALIGNMENT_UNVERIFIED\r\n",
            j->id,counts[i],(double)q[i],q[i]*180.0/3.141592653589793,
            q[i]>=j->q_min&&q[i]<=j->q_max);send_text(line);
    }
}
static void servo_calcheck(void)
{
    static const int offsets[6]={-1429,-1868,-551,1828,-773,1727};
    uint8_t bytes[4],err=0;int all_match=1;
    for(unsigned id=1;id<=6;++id){
        int rc=st_read(&servo_bus,id,9,4,bytes,&err);
        if(rc!=ST_OK){servo_report(id,rc,err);all_match=0;continue;}
        int low=bytes[0]+((int)bytes[1]<<8),high=bytes[2]+((int)bytes[3]<<8);
        rc=st_read(&servo_bus,id,31,2,bytes,&err);
        if(rc!=ST_OK){servo_report(id,rc,err);all_match=0;continue;}
        int offset=servo_word(bytes,11);
        int expected_low=id==6?1975:so101_right_joints[id-1].minimum;
        int expected_high=id==6?3567:so101_right_joints[id-1].maximum;
        int match=low==expected_low&&high==expected_high&&offset==offsets[id-1];
        char line[140];snprintf(line,sizeof line,"SERVO1 ID=%u %s min=%d max=%d offset=%d\r\n",
            id,match?"CAL_MATCH":"CAL_MISMATCH",low,high,offset);send_text(line);
        if(!match)all_match=0;
    }
    send_text(all_match?"SERVO1 CALCHECK PASS (EEPROM matches; physical alignment unverified)\r\n":
                       "SERVO1 CALCHECK FAIL\r\n");
}
/* Commands are parsed before POSE1 sequence validation and never alter pose_q. */
static void servo_command(char **t,int n)
{
    char line[256];unsigned long id;uint8_t err=0,data[15];int rc;
    if(!servo_ready){send_text("SERVO1 DISABLED\r\nSERVO1 END\r\n");return;}
    if(n==2&&!strcmp(t[1],"INFO")){
        snprintf(line,sizeof line,"SERVO1 UART1 base=0xe0001000 TX=W12 RX=V12 baud=%u SINGLE_MOTOR_JOG_AVAILABLE\r\n",servo_baud);send_text(line);
    }else if(n==2&&!strcmp(t[1],"MAP")){
        for(int i=0;i<5;++i){char line[140];const so101_joint_map *j=&so101_right_joints[i];
            snprintf(line,sizeof line,"SERVO1 ID=%d zero=%d sign=%+d min=%d max=%d verified=0\r\n",
                j->id,j->zero,j->sign,j->minimum,j->maximum);send_text(line);}
        send_text("SERVO1 ID=6 opening=0..1 count=1975..3567 verified_endpoints=0 NO_MOTION\r\n");
    }else if(n==2&&!strcmp(t[1],"ANGLES")){servo_angles();
    }else if(n==2&&!strcmp(t[1],"CALCHECK")){servo_calcheck();
    }else if(n==2&&!strcmp(t[1],"LOOPBACK")){
        uint8_t ch;for(unsigned i=0;i<128&&servo_receive(0,&ch);++i){}
        const uint8_t pattern[4]={0x55,0xa5,0x12,0x34};unsigned got=0;
        int sent=servo_send(0,pattern,4);uint32_t start=servo_ms(0);
        while(got<4&&(uint32_t)(servo_ms(0)-start)<30U){
            if(servo_receive(0,&ch)){if(ch!=pattern[got])break;++got;}
        }
        send_text(sent==4&&got==4?"SERVO1 LOOPBACK PASS\r\n":"SERVO1 LOOPBACK FAIL\r\n");
    }else if(n==2&&!strcmp(t[1],"SCAN")){
        unsigned found=0;
        for(unsigned motor=1;motor<=6;++motor){rc=st_ping(&servo_bus,motor,&err);servo_report(motor,rc,err);if(rc==ST_OK)++found;}
        snprintf(line,sizeof line,"SERVO1 FOUND=%u range=1..6\r\n",found);send_text(line);
    }else if(n==3&&!strcmp(t[1],"BAUD")&&pose_uint(t[2],&id)&&
             (id==1000000||id==500000||id==250000||id==115200)){
        if(XUartPs_SetBaudRate(&servo_uart,(u32)id)!=XST_SUCCESS)send_text("SERVO1 BAUD_ERROR\r\n");
        else {servo_baud=(unsigned)id;snprintf(line,sizeof line,"SERVO1 BAUD=%u (host UART1 only; servo EEPROM unchanged)\r\n",servo_baud);send_text(line);}
    }else if(n==3&&pose_uint(t[2],&id)&&id>=1&&id<=253){
        if(!strcmp(t[1],"PING")){rc=st_ping(&servo_bus,(unsigned)id,&err);servo_report((unsigned)id,rc,err);}
        else if(!strcmp(t[1],"OFF")){rc=st_torque_off(&servo_bus,(unsigned)id,&err);servo_report((unsigned)id,rc,err);}
        else if(!strcmp(t[1],"READ")){
            rc=st_read(&servo_bus,(unsigned)id,56,15,data,&err);
            if(rc!=ST_OK){servo_report((unsigned)id,rc,err);}
            else {
                uint8_t torque=255;rc=st_read(&servo_bus,(unsigned)id,40,1,&torque,&err);
                if(rc!=ST_OK){servo_report((unsigned)id,rc,err);}
                else {snprintf(line,sizeof line,"SERVO1 ID=%lu OK raw_pos=%d speed=%d load=%d voltage=%.1f temp=%u moving=%u current_raw=%d torque=%u\r\n",
                    id,servo_word(data,15),servo_word(data+2,15),servo_word(data+4,10),data[6]/10.0,
                    data[7],data[10],servo_word(data+13,15),torque);send_text(line);}
            }
        }else send_text("SERVO1 FORMAT\r\n");
    }else send_text("SERVO1 FORMAT\r\n");
    send_text("SERVO1 END\r\n");
}
