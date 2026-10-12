#include "arm_control.h"
#include <stdlib.h>
#include "arm_alignment.h"
static arm_controller arm;
static void arm_setup(void)
{arm_init(&arm,&servo_bus,SO101_PHYSICAL_ALIGNMENT_VERIFIED,SO101_GEOMETRY_VERIFIED);}
static const char *arm_reason(int rc)
{
    switch(rc){
    case ARM_TEMPERATURE:return "TEMPERATURE";case ARM_ALIGN:return "ALIGNMENT_UNVERIFIED";case ARM_GEOMETRY:return "GEOMETRY_UNVERIFIED";
    case ARM_TRIAL_LIMIT:return "TRIAL_TOTAL_TRAVEL";case ARM_TRIAL_TIME:return "TRIAL_TIME_LIMIT";
    case ARM_Q_LIMIT:return "MODEL_OR_ENCODER_LIMIT";case ARM_WATCHDOG:return "COMMAND_TIMEOUT";
    case ARM_STALL:return "NO_MOTION_PROGRESS";case ARM_TRACKING:return "TRACKING_ERROR";case ARM_TARGET_JUMP:return "TARGET_JUMP";
    case ARM_NO_ENABLE:return "NOT_ENABLED";default:return motor_reason(rc);
    }
}
static void arm_report(const char *kind,int rc)
{
    char line[512];int axis=0;
    if(rc==ARM_STALL)axis=arm.fault_axis;
    if(rc==ARM_TRACKING)for(int i=0;i<5;i++)if(abs(arm.actual[i]-arm.waypoint[i])>RK_ARM_TRACKING_LIMIT){axis=i+1;break;}
    snprintf(line,sizeof line,"ARM1 %s %s active=%d torque_off=%d axis=%d actual=%d,%d,%d,%d,%d waypoint=%d,%d,%d,%d,%d goal=%d,%d,%d,%d,%d\r\nARM1 END\r\n",
        kind,arm_reason(rc),arm.active,arm.off_confirmed,axis,
        arm.actual[0],arm.actual[1],arm.actual[2],arm.actual[3],arm.actual[4],
        arm.waypoint[0],arm.waypoint[1],arm.waypoint[2],arm.waypoint[3],arm.waypoint[4],
        arm.goal[0],arm.goal[1],arm.goal[2],arm.goal[3],arm.goal[4]);send_text(line);
}
static void arm_service(void)
{arm_poll(&arm);if(arm.event_pending){arm_report("EVENT",arm.outcome);arm.event_pending=0;}}
static void arm_command(char **t,int n)
{
    if(!servo_ready){arm_report("BLOCKED",ST_TX_ERROR);return;}
    if(n==2&&!strcmp(t[1],"STOP")){
        if(motor.active)motor_cancel(&motor);
        arm_stop(&arm,MOTOR_STOPPED);arm.event_pending=0;arm_report("STOP",arm.outcome);return;
    }
    if(n==2&&!strcmp(t[1],"FREEZE")){arm_fault(&arm,ARM_WATCHDOG);arm_report("FROZEN",arm.outcome);return;}
    if(n==2&&!strcmp(t[1],"RESUME")){int rc=arm_resume(&arm);arm_report(rc?"BLOCKED":"RESUMED",rc);return;}
    if(n==2&&!strcmp(t[1],"STATE")){
        char line[340];snprintf(line,sizeof line,
            "ARM1 STATE active=%d mode=%d aligned=%d geometry=%d result=%s torque_off=%d arrived=%d arrival_samples=%d frozen=%d counts=%d,%d,%d,%d,%d goal=%d,%d,%d,%d,%d\r\nARM1 END\r\n",
            arm.active,arm.mode,arm.aligned,arm.geometry,arm_reason(arm.outcome),arm.off_confirmed,arm.arrived,arm.arrival_samples,arm.frozen,
            arm.actual[0],arm.actual[1],arm.actual[2],arm.actual[3],arm.actual[4],
            arm.goal[0],arm.goal[1],arm.goal[2],arm.goal[3],arm.goal[4]);send_text(line);return;
    }
    if(motor.active){arm_report("BLOCKED",MOTOR_BUSY);return;}
    if(n==2&&!strcmp(t[1],"PAUSE")){
        int rc=arm_hold_current(&arm);
        if(!rc)memcpy(pose_q,arm.q,sizeof pose_q);
        arm_report(rc?"BLOCKED":"HELD",rc);return;
    }
    if(n==2&&(!strcmp(t[1],"CHECK")||!strcmp(t[1],"TEST_CHECK")||!strcmp(t[1],"TEST_MANUAL_CHECK"))){
        int rc=!strcmp(t[1],"TEST_MANUAL_CHECK")?arm_manual_check(&arm):!strcmp(t[1],"TEST_CHECK")?arm_trial_check(&arm):arm_check(&arm);
        if(!rc)memcpy(pose_q,arm.q,sizeof pose_q);
        arm_report(rc?"BLOCKED":"CHECK_OK",rc);return;
    }
    if(n==2&&!strcmp(t[1],"KEEP")){
        if(arm.frozen){arm_report("FROZEN",arm.outcome);return;}
        int rc=arm_keep(&arm);arm_report(rc?"BLOCKED":"ALIVE",rc);return;
    }
    if(n==2&&(!strcmp(t[1],"ENABLE")||!strcmp(t[1],"HOLD")||!strcmp(t[1],"TEST_ENABLE")||!strcmp(t[1],"TEST_MANUAL_ENABLE"))){
        int hold=!strcmp(t[1],"TEST_MANUAL_ENABLE")?3:!strcmp(t[1],"TEST_ENABLE")?2:!strcmp(t[1],"HOLD"),rc=arm_begin(&arm,hold);arm.event_pending=0;
        if(rc){arm_report("BLOCKED",rc);return;}
        if(hold!=1)memcpy(pose_q,arm.q,sizeof pose_q);
        send_text(hold==3?"ARM1 READY MANUAL_OUT_HOLD_V2 UNVERIFIED_MAPPING COORDINATED_MANUAL_V1 SYNC_TARGET_HOLD_V1 LIMITS600_TEMP1500_20261011_V1 SPEED_PARAMETER_MIN=100 SPEED_PARAMETER_MAX=600 TEMP_LIMIT_C=55 TEMP_FILTER_MS=1500 LOCAL_ENVELOPE=OFF TORQUE_CAP=500 ARRIVAL_COUNTS=8 ARRIVAL_SAMPLES=3 NO_PROGRESS_STOP=OFF WATCHDOG_MS=500 TORQUE_ON_VERIFIED=1,2,3,4,5 GRIPPER_OFF\r\nARM1 END\r\n":hold==2?"ARM1 READY LIMITED_DRIVE_V1 UNVERIFIED_MAPPING MAX_COUNTS=56 MAX_MS=10000 GRIPPER_OFF\r\nARM1 END\r\n":hold?"ARM1 HOLDING CURRENT_COUNTS GRIPPER_OFF\r\nARM1 END\r\n":
                       "ARM1 READY FEEDBACK_IK_SYNC_WRITE GRIPPER_OFF\r\nARM1 END\r\n");return;
    }
    arm_report("ERROR",ST_ARGUMENT);
}
static int arm_step(void *ctx,const float q[5],const float e[6],float d,float out[5])
{
    if(arm.active&&(uint32_t)(servo_ms(0)-arm.last_command)>RK_ARM_WATCHDOG_MS)return 1;
    return hardware_step(ctx,q,e,d,out);
}
