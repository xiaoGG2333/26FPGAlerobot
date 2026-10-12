/* Full ARM iteration; verified arm mode queues bounded servo trajectories. */
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <limits.h>
static const float pose_home[5]={.2f,-.3f,.4f,-.2f,.15f};
static float pose_q[5]={.2f,-.3f,.4f,-.2f,.15f};
static float pose_gripper=.5f;
static unsigned long pose_last_sequence=0;
static void pose_state(unsigned long seq)
{
    double T[16];char reply[384];ik_fk(pose_q,T);
    double pitch=atan2(-T[8],hypot(T[0],T[4]));
    double roll=atan2(T[9],T[10]),yaw=atan2(T[4],T[0]);
    snprintf(reply,sizeof reply,"STATE1 %lu %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %lu\r\n",seq,
        (double)pose_q[0],(double)pose_q[1],(double)pose_q[2],(double)pose_q[3],(double)pose_q[4],
        T[3],T[7],T[11],roll,pitch,yaw,pose_last_sequence);send_text(reply);
}
static void pose_error(unsigned long seq,const char *reason)
{char line[96];snprintf(line,sizeof line,"ERR1 %lu %s\r\n",seq,reason);send_text(line);}
static int pose_uint(const char *s,unsigned long *out)
{
    char *end;if(!*s)return 0;
    for(const char *p=s;*p;++p)if(*p<'0'||*p>'9')return 0;
    errno=0;*out=strtoul(s,&end,10);
    return !errno && !*end && *out<=UINT32_MAX;
}
#include "servo_uart.inc"
#include "motor_uart.inc"
#include "arm_uart.inc"
#include "cal_uart.inc"
static void pose_counts(unsigned long seq)
{
    int counts[5]={0};int rc=so101_q_to_counts(pose_q,counts);
    int gripper=(int)lroundf(1975+pose_gripper*(3567-1975));char reply[200];
    snprintf(reply,sizeof reply,"COUNT1 %lu %d %d %d %d %d %d %d %d\r\n",seq,rc,
        counts[0],counts[1],counts[2],counts[3],counts[4],gripper,SO101_PHYSICAL_ALIGNMENT_VERIFIED);
    send_text(reply);
}
static void pose_line(char *line)
{
    char *tokens[12],*t=strtok(line," \t\r");int n=0;
    while(t && n<12){tokens[n++]=t;t=strtok(NULL," \t\r");}
    if(t){pose_error(0,"FORMAT");return;}
    if(n>=1&&!strcmp(tokens[0],"CAL1")){cal_command(tokens,n);return;}
    if(n==1&&!strcmp(tokens[0],"HELLO")){
        if(calibration.active)cal_keep(&calibration);
        send_text("POSE1 READY RK7020 RIGHT_ARM ARM_CONTROL_V1 AUTOCAL_BRIDGE_V1 ATOMIC_HOLD BUS_TRACE_V1 SO101_135MM LIMITED_DRIVE_V1 MANUAL_OUT_HOLD_V2 TRACKING_DIAG_V1 BUTTONS_INPUT_V1 STICK_INPUT_V1 COORDINATED_MANUAL_V1 OFFICIAL_R12254604_V1 MANUAL_REFERENCE_20261009_V1 MOTOR_STAGE_DIAG_V1 JOG_WRIST_20DEG_V1 TORQUE_SETTLE_150MS_V1 SYNC_TARGET_HOLD_V1 LIMITS600_TEMP1500_20261011_V1\r\n");return;
    }
    if(calibration.active||calibration.fault){cal_error(0,CAL_BUSY);return;}
    if(n>=1&&!strcmp(tokens[0],"ARM1")){arm_command(tokens,n);return;}
    if(n>=1&&!strcmp(tokens[0],"MOTOR1")){
        if(n==2&&!strcmp(tokens[1],"STOP")&&arm.active){arm_stop(&arm,MOTOR_STOPPED);arm.event_pending=0;}
        else if(arm.active){send_text("MOTOR1 ERROR ARM_BUSY\r\nMOTOR1 END\r\n");return;}
        motor_command(tokens,n);return;
    }
    if(motor.active){
        if(n>=1&&!strcmp(tokens[0],"SERVO1"))send_text("SERVO1 BUSY\r\nSERVO1 END\r\n");
        else pose_error(0,"MOTOR_BUSY");
        return;
    }
    if(n>=1 && !strcmp(tokens[0],"SERVO1")){
        if(arm.active){send_text("SERVO1 BUSY\r\nSERVO1 END\r\n");return;}
        servo_command(tokens,n);return;
    }
    if(n==1 && !strcmp(tokens[0],"HELLO")){
        send_text("POSE1 READY RK7020 RIGHT_ARM ARM_CONTROL_V1 AUTOCAL_BRIDGE_V1 ATOMIC_HOLD BUS_TRACE_V1 SO101_135MM LIMITED_DRIVE_V1 MANUAL_OUT_HOLD_V2 TRACKING_DIAG_V1 BUTTONS_INPUT_V1 STICK_INPUT_V1 COORDINATED_MANUAL_V1 OFFICIAL_R12254604_V1 MANUAL_REFERENCE_20261009_V1 MOTOR_STAGE_DIAG_V1 JOG_WRIST_20DEG_V1 TORQUE_SETTLE_150MS_V1 SYNC_TARGET_HOLD_V1 LIMITS600_TEMP1500_20261011_V1\r\n");return;
    }
    unsigned long seq=0,core;
    if(n<2 || !pose_uint(tokens[1],&seq)){pose_error(0,"SEQUENCE");return;}
    if(n==2 && !strcmp(tokens[0],"STATE1")){
        if(arm.active&&(arm.mode==2||arm.mode==3)){
            int rc=arm_refresh(&arm);if(rc){arm_fault(&arm,rc);pose_error(seq,"FEEDBACK_ERROR");return;}
            memcpy(pose_q,arm.q,sizeof pose_q);
        }
        pose_state(seq);return;
    }
    if(n==2 && !strcmp(tokens[0],"COUNTS1")){pose_counts(seq);return;}
    if(seq==0 || seq<=pose_last_sequence){pose_error(seq,"STALE_SEQUENCE");return;}
    if(n==2 && !strcmp(tokens[0],"RESET1")){
        if(arm.active){pose_error(seq,"ARM_ACTIVE_NO_RESET");return;}
        pose_last_sequence=seq;memcpy(pose_q,pose_home,sizeof pose_q);pose_gripper=.5f;pose_state(seq);return;
    }
    if(n!=10 || strcmp(tokens[0],"POSE1")){pose_error(seq,"FORMAT");return;}
    if(!pose_uint(tokens[2],&core)||core>1){pose_error(seq,"CORE");return;}
    float v[7];char *end;
    for(int i=0;i<7;++i){
        errno=0;v[i]=strtof(tokens[i+3],&end);
        if(errno||!*tokens[i+3]||*end||!isfinite(v[i])){pose_error(seq,"VALUE");return;}
    }
    /* Broad model envelope only; not a physical-arm safety workspace. */
    if(v[0]<-.5f||v[0]>.6f||fabsf(v[1])>.6f||v[2]<-.3f||v[2]>.7f||
       fabsf(v[3])>3.141593f||fabsf(v[4])>1.4f||fabsf(v[5])>3.141593f||v[6]<0||v[6]>1){
        pose_error(seq,"RANGE");return;
    }
    pose_last_sequence=seq;
    if(arm.active){
        if(arm.frozen){pose_error(seq,"ARM_FROZEN");return;}
        if(arm.mode!=2&&arm.mode!=3){pose_error(seq,"HOLD_MODE_NO_POSE");return;}
        int rc=arm_refresh(&arm);if(rc){arm_fault(&arm,rc);pose_error(seq,"FEEDBACK_ERROR");return;}
        memcpy(pose_q,arm.q,sizeof pose_q);
    }
    double cr=cos(v[3]),sr=sin(v[3]),cp=cos(v[4]),sp=sin(v[4]),cy=cos(v[5]),sy=sin(v[5]);
    /* Intrinsic xyz / Rz(yaw) Ry(pitch) Rx(roll), SI units, row-major. */
    double target[16]={cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr,v[0],
        sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr,v[1],-sp,cp*sr,cp*cr,v[2],0,0,0,1};
    ik_result result;int selected=(int)core;
    int code=ik_solve(target,pose_q,200,1e-8,.01f,1,arm_step,NULL,&selected,&result);
    if(code==0)for(int i=0;i<5;++i)if(result.q[i]<so101_right_joints[i].q_min||result.q[i]>so101_right_joints[i].q_max)code=4;
    if(code==0){int preview[5];if(so101_q_to_counts(result.q,preview)!=0)code=4;}
    if(arm.active){
        if(code){send_text("LeRobot ERROR REJECTED KEEP_LAST_GOAL TORQUE_UNCHANGED\r\n");}
        else {int rc=arm_target(&arm,result.q);if(rc){code=rc==ARM_Q_LIMIT?4:2;if(rc==ARM_Q_LIMIT&&arm.active&&arm.manual_session)send_text("LeRobot OUT HOLD_CURRENT TORQUE_ON\r\n");}}
    }
    /* Commit only successful results. Failure replies carry candidate q for diagnostics. */
    if(code==0){memcpy(pose_q,result.q,sizeof pose_q);pose_gripper=v[6];}
    char reply[384];
    snprintf(reply,sizeof reply,"RESULT1 %lu %d %d %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\r\n",
        seq,code,result.iterations,result.position_error,result.rotation_error,
        (double)result.q[0],(double)result.q[1],(double)result.q[2],(double)result.q[3],(double)result.q[4],(double)v[6]);
    send_text(reply);
}
static void pose_server(void)
{
    char line[256];unsigned int used=0;int discard=0;u8 ch;
    send_text("POSE1 READY RK7020 RIGHT_ARM ARM_CONTROL_V1 AUTOCAL_BRIDGE_V1 ATOMIC_HOLD BUS_TRACE_V1 SO101_135MM LIMITED_DRIVE_V1 MANUAL_OUT_HOLD_V2 TRACKING_DIAG_V1 BUTTONS_INPUT_V1 STICK_INPUT_V1 COORDINATED_MANUAL_V1 OFFICIAL_R12254604_V1 MANUAL_REFERENCE_20261009_V1 MOTOR_STAGE_DIAG_V1 JOG_WRIST_20DEG_V1 TORQUE_SETTLE_150MS_V1 SYNC_TARGET_HOLD_V1 LIMITS600_TEMP1500_20261011_V1\r\n");
    for(;;){
        cal_service();
        motor_service();
        arm_service();
        if(!XUartPs_Recv(&uart,&ch,1)){usleep(100);continue;}
        if(ch=='\n'){
            if(discard)pose_error(0,"LINE_TOO_LONG");
            else {line[used]=0;pose_line(line);}
            used=0;discard=0;
        }else if(ch!='\r'&&!discard){
            if(used<sizeof line-1)line[used++]=(char)ch;else discard=1;
        }
    }
}
