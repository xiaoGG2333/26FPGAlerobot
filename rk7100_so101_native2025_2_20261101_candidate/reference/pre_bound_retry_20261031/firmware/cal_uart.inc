#include "cal_bridge.h"
static cal_controller calibration;
static void cal_setup(void){cal_init(&calibration,&servo_bus);}
static const char *cal_reason(int rc)
{
    switch(rc){
    case CAL_INACTIVE:return "INACTIVE";case CAL_BUSY:return "BUSY";
    case CAL_STALE:return "STALE_SEQUENCE";case CAL_FORBIDDEN:return "FORBIDDEN";
    case CAL_TORQUE_ACTIVE:return "TORQUE_ACTIVE";case CAL_ENABLE_GUARD:return "ENABLE_GUARD";
    case CAL_HEALTH:return "HEALTH";case CAL_SECOND_VELOCITY:return "SECOND_VELOCITY";
    case CAL_CLEANUP:return "CLEANUP_NOT_CONFIRMED";case CAL_FAULT:return "FAULT_LATCHED";
    default:return servo_reason(rc);
    }
}
static void cal_error(unsigned long seq,int rc)
{
    char reply[260];snprintf(reply,sizeof reply,
        "CAL1 ERROR %lu %s active=%d off_confirmed=%d rollback_success=%d first_fail_id=%u first_fail_addr=%u first_fail_rc=%d\r\nCAL1 END\r\n",
        seq,cal_reason(rc),calibration.active,calibration.off_confirmed,calibration.rollback_success,
        calibration.first_fail_id,calibration.first_fail_addr,calibration.first_fail_rc);
    send_text(reply);
}
static void cal_service(void)
{
    cal_poll(&calibration);
    if(calibration.event_pending){
        char reply[230];snprintf(reply,sizeof reply,
            "CAL1 EVENT WATCHDOG off_confirmed=%d rollback_success=%d first_fail_id=%u first_fail_addr=%u first_fail_rc=%d\r\nCAL1 END\r\n",
            calibration.off_confirmed,calibration.rollback_success,
            calibration.first_fail_id,calibration.first_fail_addr,calibration.first_fail_rc);send_text(reply);
        calibration.event_pending=0;
    }
}
static int cal_hex(char ch)
{if(ch>='0'&&ch<='9')return ch-'0';if(ch>='a'&&ch<='f')return ch-'a'+10;if(ch>='A'&&ch<='F')return ch-'A'+10;return -1;}
static void cal_command(char **t,int n)
{
    char reply[768];int rc;unsigned long seq=0,id,a,count;
    if(n==3&&!strcmp(t[1],"TRACE")){
        if(calibration.active||motor.active||arm.active){cal_error(0,CAL_BUSY);return;}
        if(!strcmp(t[2],"COUNT")){
            snprintf(reply,sizeof reply,"CAL1 TRACE COUNT %u\r\nCAL1 END\r\n",st_trace_count());
            send_text(reply);return;
        }
        unsigned long index;st_trace_record record;
        if(!pose_uint(t[2],&index)||index>=st_trace_count()||
           st_trace_get((unsigned)index,&record)!=ST_OK){cal_error(0,CAL_FORBIDDEN);return;}
        static const char hex[]="0123456789ABCDEF";
        char tx[2*ST_TRACE_TX_MAX+1],rx[2*ST_TRACE_RX_MAX+1];
        for(unsigned i=0;i<record.tx_length;++i){tx[2*i]=hex[record.tx[i]>>4];tx[2*i+1]=hex[record.tx[i]&15];}
        for(unsigned i=0;i<record.rx_length;++i){rx[2*i]=hex[record.rx[i]>>4];rx[2*i+1]=hex[record.rx[i]&15];}
        tx[2*record.tx_length]=0;rx[2*record.rx_length]=0;
        snprintf(reply,sizeof reply,
            "CAL1 TRACE %lu id=%u op=%u addr=%u count=%u rc=%d drain=%u truncated=%d start_ms=%lu end_ms=%lu elapsed_ms=%lu tx=%s rx=%s\r\nCAL1 END\r\n",
            index,record.id,record.op,record.address,record.count,record.rc,record.drain,record.truncated,
            (unsigned long)record.start_ms,(unsigned long)record.end_ms,
            (unsigned long)(uint32_t)(record.end_ms-record.start_ms),tx,rx);
        send_text(reply);return;
    }
    if(!servo_ready){cal_error(0,ST_TX_ERROR);return;}
    if(n==2&&!strcmp(t[1],"BEGIN")){
        if(motor.active||arm.active){cal_error(0,CAL_BUSY);return;}
        rc=cal_begin(&calibration);
        if(rc){cal_error(0,rc);return;}
        send_text("CAL1 READY active=1 watchdog_ms=2000\r\nCAL1 END\r\n");return;
    }
    if(n==2&&!strcmp(t[1],"STATE")){
        if(calibration.active)cal_keep(&calibration);
        snprintf(reply,sizeof reply,"CAL1 STATE active=%d last_seq=%lu fault=%d off_confirmed=%d rollback_success=%d\r\nCAL1 END\r\n",
            calibration.active,(unsigned long)calibration.last_seq,calibration.fault,
            calibration.off_confirmed,calibration.rollback_success);send_text(reply);return;
    }
    if(n==2&&!strcmp(t[1],"KEEP")){
        rc=cal_keep(&calibration);if(rc){cal_error(0,rc);return;}
        send_text("CAL1 KEEP active=1\r\nCAL1 END\r\n");return;
    }
    if(n==2&&(!strcmp(t[1],"STOP")||!strcmp(t[1],"COMMIT"))){
        int commit=!strcmp(t[1],"COMMIT");
        if(motor.active||arm.active){cal_error(0,CAL_BUSY);return;}
        if(commit&&(!calibration.active||calibration.fault)){cal_error(0,CAL_INACTIVE);return;}
        rc=cal_finish(&calibration,commit);calibration.event_pending=0;
        snprintf(reply,sizeof reply,"CAL1 %s off_confirmed=%d rollback_success=%d result=%s first_fail_id=%u first_fail_addr=%u first_fail_rc=%d\r\nCAL1 END\r\n",
            commit?"COMMIT":"STOP",calibration.off_confirmed,calibration.rollback_success,cal_reason(rc),
            calibration.first_fail_id,calibration.first_fail_addr,calibration.first_fail_rc);
        send_text(reply);return;
    }
    if(n==4&&!strcmp(t[1],"HOLD")){
        if(!pose_uint(t[2],&seq)||!pose_uint(t[3],&id)){cal_error(0,CAL_FORBIDDEN);return;}
        rc=cal_hold(&calibration,(uint32_t)seq,(unsigned)id);
        if(rc){cal_error(seq,rc);return;}
        snprintf(reply,sizeof reply,"CAL1 OK %lu\r\nCAL1 END\r\n",seq);send_text(reply);return;
    }
    if(n!=6||!pose_uint(t[2],&seq)||!pose_uint(t[3],&id)||!pose_uint(t[4],&a)){
        cal_error(0,CAL_FORBIDDEN);return;
    }
    uint8_t bytes[32];
    if(!strcmp(t[1],"READ")&&pose_uint(t[5],&count)){
        rc=cal_read(&calibration,(uint32_t)seq,(unsigned)id,(unsigned)a,(unsigned)count,bytes);
        if(rc){cal_error(seq,rc);return;}
        static const char hex[]="0123456789ABCDEF";char data[65];
        for(unsigned i=0;i<count;++i){data[2*i]=hex[bytes[i]>>4];data[2*i+1]=hex[bytes[i]&15];}
        data[2*count]=0;snprintf(reply,sizeof reply,"CAL1 DATA %lu %s\r\nCAL1 END\r\n",seq,data);send_text(reply);return;
    }
    if(!strcmp(t[1],"WRITE")){
        size_t len=strlen(t[5]);if(len!=2&&len!=4){cal_error(seq,CAL_FORBIDDEN);return;}
        count=(unsigned long)(len/2);
        for(unsigned i=0;i<count;++i){int x=cal_hex(t[5][2*i]),y=cal_hex(t[5][2*i+1]);
            if(x<0||y<0){cal_error(seq,CAL_FORBIDDEN);return;}bytes[i]=(uint8_t)(16*x+y);}
        rc=cal_write(&calibration,(uint32_t)seq,(unsigned)id,(unsigned)a,(unsigned)count,bytes);
        if(rc){cal_error(seq,rc);return;}
        snprintf(reply,sizeof reply,"CAL1 OK %lu\r\nCAL1 END\r\n",seq);send_text(reply);return;
    }
    cal_error(seq,CAL_FORBIDDEN);
}
