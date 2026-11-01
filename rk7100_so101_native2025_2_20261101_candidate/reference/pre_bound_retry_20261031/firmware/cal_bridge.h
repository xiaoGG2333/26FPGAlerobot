#ifndef RK_CAL_BRIDGE_H
#define RK_CAL_BRIDGE_H
#include "st3215.h"
#define CAL_WATCHDOG_MS 2000U
enum { CAL_INACTIVE=40,CAL_BUSY=41,CAL_STALE=42,CAL_FORBIDDEN=43,
       CAL_TORQUE_ACTIVE=44,CAL_ENABLE_GUARD=45,CAL_HEALTH=46,
       CAL_SECOND_VELOCITY=47,CAL_CLEANUP=48,CAL_FAULT=49 };
typedef struct {
    st_bus *bus;
    int active,snapshot_valid,fault,off_confirmed,rollback_success,event_pending;
    unsigned moving_id;
    unsigned first_fail_id,first_fail_addr;int first_fail_rc;
    uint32_t last_seq,last_rpc;
    uint8_t saved[6][40],saved_lock[6],goal_ready[6],error;
} cal_controller;
void cal_init(cal_controller *,st_bus *);
int cal_begin(cal_controller *);
int cal_read(cal_controller *,uint32_t seq,unsigned id,unsigned addr,unsigned count,uint8_t *);
int cal_write(cal_controller *,uint32_t seq,unsigned id,unsigned addr,unsigned count,const uint8_t *);
/* Current-position hold only; there is no caller-supplied position target. */
int cal_hold(cal_controller *,uint32_t seq,unsigned id);
int cal_keep(cal_controller *);
int cal_finish(cal_controller *,int commit);
void cal_poll(cal_controller *);
#endif
