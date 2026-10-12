#ifndef RK_MOTOR_CONTROL_H
#define RK_MOTOR_CONTROL_H
#include "st3215.h"
enum { MOTOR_BUSY=10,MOTOR_ARGUMENT=11,MOTOR_LIMIT=12,MOTOR_CALIBRATION=13,
       MOTOR_HEALTH=14,MOTOR_TORQUE_ACTIVE=15,MOTOR_STALE=16,MOTOR_STOPPED=17,MOTOR_STALLED=18 };
typedef struct {
    st_bus *bus;
    const char *stage;
    unsigned id;uint32_t last_sequence,start_ms,last_poll;
    int active,done_pending,origin,target,position,outcome,stable;
    int off_confirmed;uint8_t error;
} motor_controller;
void motor_init(motor_controller *,st_bus *);
int motor_jog(motor_controller *,uint32_t sequence,unsigned id,int delta);
void motor_poll(motor_controller *);
void motor_cancel(motor_controller *);
#endif
