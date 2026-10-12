#ifndef RK_ARM_CONTROL_H
#define RK_ARM_CONTROL_H
#include "st3215.h"
enum {ARM_ALIGN=20,ARM_GEOMETRY=21,ARM_Q_LIMIT=22,ARM_WATCHDOG=23,
      ARM_TRACKING=24,ARM_TARGET_JUMP=25,ARM_NO_ENABLE=26,ARM_TRIAL_LIMIT=27,ARM_TRIAL_TIME=28,ARM_STALL=29,ARM_TEMPERATURE=30};
typedef struct {
    st_bus *bus;
    int aligned,geometry,active,mode,outcome,off_confirmed,event_pending,manual_session,fault_axis;
    int frozen,valid_goal;
    int arrived,arrival_samples;unsigned goal_speeds[5];
    int actual[5],goal[5],waypoint[5];float q[5];uint8_t error;
    uint32_t last_command,last_poll,trial_started;
    uint32_t temperature_since[5];uint8_t temperature_pending[5];
    uint32_t progress_ms[5];int progress_counts[5];
    int trial_origin[5];unsigned saved_caps[5],cap_mask;
} arm_controller;
void arm_init(arm_controller *,st_bus *,int aligned,int geometry);
int arm_check(arm_controller *);
int arm_trial_check(arm_controller *);
int arm_manual_check(arm_controller *);
int arm_begin(arm_controller *,int hold_only);
int arm_refresh(arm_controller *);
int arm_hold_current(arm_controller *);
int arm_target(arm_controller *,const float q[5]);
int arm_keep(arm_controller *);
void arm_fault(arm_controller *,int outcome);
int arm_resume(arm_controller *);
void arm_stop(arm_controller *,int outcome);
void arm_poll(arm_controller *);
#endif
