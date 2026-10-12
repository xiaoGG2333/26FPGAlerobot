#include "motor_control.h"
static motor_controller motor;
static int motor_configured=0;
static const char *motor_reason(int rc)
{
    if(rc<=6)return servo_reason(rc);
    switch(rc){
    case MOTOR_BUSY:return "BUSY";case MOTOR_ARGUMENT:return "ARGUMENT";
    case MOTOR_LIMIT:return "LIMIT";case MOTOR_CALIBRATION:return "CALIBRATION_MISMATCH";
    case MOTOR_HEALTH:return "HEALTH_OR_POSITION";case MOTOR_TORQUE_ACTIVE:return "TORQUE_ALREADY_ACTIVE";
    case MOTOR_STALE:return "STALE_SEQUENCE";case MOTOR_STOPPED:return "STOPPED";
    case MOTOR_STALLED:return "MOTION_TIMEOUT";default:return "ERROR";
    }
}
static void motor_setup(void)
{motor_init(&motor,&servo_bus);motor_configured=servo_ready;}
static void motor_terminal(void)
{
    if(!motor.done_pending)return;
    char line[220];snprintf(line,sizeof line,
        "MOTOR1 DONE seq=%lu id=%u result=%s origin=%d target=%d position=%d torque_off=%d stage=%s\r\nMOTOR1 END\r\n",
        (unsigned long)motor.last_sequence,motor.id,motor_reason(motor.outcome),motor.origin,motor.target,
        motor.position,motor.off_confirmed,motor.stage);send_text(line);motor.done_pending=0;
}
static void motor_service(void)
{if(motor_configured){motor_poll(&motor);motor_terminal();}}
static int motor_delta(const char *s,int *value)
{
    char *end;const char *p=s;if(*p=='+'||*p=='-')++p;
    if(!*p)return 0;
    for(;*p;++p)if(*p<'0'||*p>'9')return 0;
    errno=0;long n=strtol(s,&end,10);
    if(errno||*end||n<-240||n>240||labs(n)<10)return 0;
    *value=(int)n;return 1;
}
static void motor_command(char **t,int n)
{
    if(!motor_configured){send_text("MOTOR1 ERROR DISABLED\r\nMOTOR1 END\r\n");return;}
    if(n==2&&!strcmp(t[1],"STATE")){
        char line[200];snprintf(line,sizeof line,
            "MOTOR1 STATE active=%d last_seq=%lu id=%u position=%d target=%d result=%s torque_off=%d stage=%s\r\nMOTOR1 END\r\n",
            motor.active,(unsigned long)motor.last_sequence,motor.id,motor.position,motor.target,
            motor_reason(motor.outcome),motor.off_confirmed,motor.stage);send_text(line);return;
    }
    if(n==2&&!strcmp(t[1],"STOP")){
        motor_cancel(&motor);motor.done_pending=0;
        int all_off=1;uint8_t err=0;
        for(unsigned id=1;id<=6;++id){
            int rc=st_torque_off(&servo_bus,id,&err);
            if(rc){all_off=0;servo_report(id,rc,err);}
        }
        send_text(all_off?"MOTOR1 STOP ALL_TORQUE_OFF\r\nMOTOR1 END\r\n":
                          "MOTOR1 STOP OFF_NOT_CONFIRMED\r\nMOTOR1 END\r\n");return;
    }
    unsigned long seq,id;int delta;
    if(n!=5||strcmp(t[1],"JOG")||!pose_uint(t[2],&seq)||!pose_uint(t[3],&id)||
       id<1||id>6||!motor_delta(t[4],&delta)){
        send_text("MOTOR1 ERROR FORMAT expected=JOG_seq_id_delta_ID5_10..240_other_10..100\r\nMOTOR1 END\r\n");return;
    }
    int rc=motor_jog(&motor,(uint32_t)seq,(unsigned)id,delta);
    if(rc){
        if(motor.done_pending){motor_terminal();}
        else {char line[120];snprintf(line,sizeof line,"MOTOR1 ERROR seq=%lu %s\r\nMOTOR1 END\r\n",seq,motor_reason(rc));send_text(line);}
    }else{
        char line[160];snprintf(line,sizeof line,
            "MOTOR1 ACCEPTED seq=%lu id=%lu origin=%d target=%d speed=%u delta=%d\r\n",
            seq,id,motor.origin,motor.target,id==5?100U:50U,delta);send_text(line);
        /* Completion END comes from service while UART remains responsive. */
    }
}
