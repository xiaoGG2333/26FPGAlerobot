#ifndef RK_ST3215_H
#define RK_ST3215_H
#include <stdint.h>
typedef struct {
    void *context;
    int (*send)(void *,const uint8_t *,unsigned);
    int (*receive)(void *,uint8_t *);
    uint32_t (*milliseconds)(void *);
} st_bus;
enum { ST_OK=0, ST_TIMEOUT=1, ST_CHECKSUM=2, ST_FORMAT=3,
       ST_TX_ERROR=4, ST_DEVICE_ERROR=5, ST_ARGUMENT=6 };
/* Diagnostic-only transaction history. TRACE never sends or receives bus data. */
#define ST_TRACE_CAPACITY 32U
#define ST_TRACE_TX_MAX 16U
#define ST_TRACE_RX_MAX 96U
typedef struct {
    uint32_t start_ms,end_ms;
    unsigned id,op,address,count,drain,tx_length,rx_length;
    int rc,truncated;
    uint8_t tx[ST_TRACE_TX_MAX],rx[ST_TRACE_RX_MAX];
} st_trace_record;
void st_trace_clear(void); /* Also unfreezes, for a new calibration session. */
void st_trace_freeze(void); /* Idempotent; cleanup must not erase fault evidence. */
unsigned st_trace_count(void);
int st_trace_get(unsigned index,st_trace_record *record); /* Oldest -> newest. */
/* Single transaction, bounded 30ms deadline. No EEPROM writes. */
int st_ping(st_bus *,unsigned id,uint8_t *error);
int st_read(st_bus *,unsigned id,unsigned address,unsigned count,uint8_t *data,uint8_t *error);
int st_torque_off(st_bus *,unsigned id,uint8_t *error);
int st_torque_on(st_bus *,unsigned id,uint8_t *error);
int st_set_goal(st_bus *,unsigned id,unsigned position,unsigned speed,unsigned acceleration,uint8_t *error);
/* RAM-only Sync Write to the five arm IDs 1..5; then individual readback. */
int st_sync_arm_goals(st_bus *,const int positions[5],unsigned speed,unsigned acceleration,uint8_t *error);
/* One broadcast goal with per-axis speed, RAM only. Speeds 1..200. */
int st_sync_arm_goals_speeds(st_bus *,const int positions[5],const unsigned speeds[5],unsigned acceleration,uint8_t *error);
/* Calibration-only register whitelist. One WRITE, followed by readback; no retry. */
int st_cal_write(st_bus *,unsigned id,unsigned address,unsigned count,const uint8_t *,uint8_t *error);
#endif
