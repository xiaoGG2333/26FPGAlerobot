#ifndef RK_GRIPPER_CONTROL_H
#define RK_GRIPPER_CONTROL_H
#include "st3215.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
enum {GP_LIMIT=40,GP_TEMPERATURE=41,GP_NOT_ENABLED=42,GP_WATCHDOG=43,GP_CONFIG=44};
typedef struct {
 st_bus *bus;int configured,active,valid,frozen,reason,direction,actual,goal,closed,opened,raw_low,raw_high;
 int pending_temperature,cap_saved,off_confirmed;unsigned speed,rate,cap,old_cap;uint8_t error;
 uint32_t last_command,last_poll,last_step,temperature_since;double requested;
} grip_controller;
static inline uint32_t grip_now(grip_controller *g){return g->bus->milliseconds(g->bus->context);}
static inline int grip_word(const uint8_t *p){return p[0]+((int)p[1]<<8);}
static inline void grip_init(grip_controller *g,st_bus *bus){memset(g,0,sizeof *g);g->bus=bus;g->raw_high=4095;}
static inline int grip_bounds(grip_controller *g,int p){int lo=g->closed<g->opened?g->closed:g->opened,hi=g->closed>g->opened?g->closed:g->opened;return p>=lo&&p<=hi;}
static inline int grip_config(grip_controller *g,int closed,int opened,unsigned rate,unsigned speed,unsigned cap){
 if(g->active||g->cap_saved||closed<0||closed>4095||opened<0||opened>4095||abs(closed-opened)<64||rate<1||rate>400||speed<1||speed>100||cap<1||cap>500)return GP_CONFIG;
 g->closed=closed;g->opened=opened;g->rate=rate;g->speed=speed;g->cap=cap;g->configured=1;return 0;
}
static inline int grip_read(grip_controller *g){
 uint8_t p[15];int rc=st_read(g->bus,6,56,15,p,&g->error);if(rc){g->pending_temperature=0;return rc;}
 int actual=grip_word(p);uint32_t now=grip_now(g);
 if(p[7]>55){if(!g->pending_temperature){g->pending_temperature=1;g->temperature_since=now;}
 else if((uint32_t)(now-g->temperature_since)>1500U)return GP_TEMPERATURE;
 }else g->pending_temperature=0;
 if((actual&32768)||(actual<g->raw_low||actual>g->raw_high)||p[6]<90||p[6]>126)return GP_LIMIT;
 g->actual=actual;return 0;
}
static inline void grip_freeze(grip_controller *g,int reason){g->direction=0;g->frozen=1;g->reason=reason;/* No torque or goal writes. */}
static inline int grip_end(grip_controller *g){
 int rc=ST_TIMEOUT;g->off_confirmed=0;g->active=0;g->valid=0;g->direction=0;g->frozen=0;
 for(int n=0;n<3&&rc;n++)rc=st_torque_off(g->bus,6,&g->error);
 if(!rc)g->off_confirmed=1;
 if(!rc&&g->cap_saved){uint8_t p[2]={(uint8_t)g->old_cap,(uint8_t)(g->old_cap>>8)};rc=st_cal_write(g->bus,6,48,2,p,&g->error);if(!rc)g->cap_saved=0;}
 g->reason=rc;return rc;
}
static inline int grip_begin(grip_controller *g){
 if(!g->configured||g->active||g->cap_saved)return GP_CONFIG;
 int rc;uint8_t limits[4];rc=st_read(g->bus,6,9,4,limits,&g->error);
 if(!rc){g->raw_low=grip_word(limits);g->raw_high=grip_word(limits+2);}
 if(!rc&&(g->raw_low>=g->raw_high||g->raw_high>4095||g->closed<g->raw_low||g->closed>g->raw_high||g->opened<g->raw_low||g->opened>g->raw_high))rc=GP_CONFIG;
 if(!rc)rc=grip_read(g);
 uint8_t p[2];if(!rc)rc=st_read(g->bus,6,48,2,p,&g->error);
 if(!rc){g->old_cap=(unsigned)grip_word(p);if(!g->old_cap||g->old_cap>1000)rc=GP_CONFIG;}
 if(!rc){unsigned cap=g->old_cap<g->cap?g->old_cap:g->cap;g->cap_saved=1;p[0]=(uint8_t)cap;p[1]=(uint8_t)(cap>>8);rc=st_cal_write(g->bus,6,48,2,p,&g->error);}
 if(!rc){g->off_confirmed=0;rc=st_set_goal(g->bus,6,(unsigned)g->actual,g->speed,5,&g->error);}
 if(!rc){g->goal=g->actual;g->requested=g->actual;rc=st_torque_on(g->bus,6,&g->error);}
 if(rc){grip_end(g);return rc;}
 g->active=1;g->valid=1;g->frozen=0;g->reason=0;g->direction=0;g->last_command=g->last_poll=g->last_step=grip_now(g);return 0;
}
static inline int grip_input(grip_controller *g,int direction){
 if(direction< -1||direction>1)return ST_ARGUMENT;
 if(!g->active||!g->valid)return GP_NOT_ENABLED;
 if(g->frozen)return g->reason;
 g->last_command=grip_now(g);
 if(direction!=g->direction){
  int rc=grip_read(g);if(rc){grip_freeze(g,rc);return rc;}
  rc=st_set_goal(g->bus,6,(unsigned)g->actual,g->speed,5,&g->error);
  if(rc){grip_freeze(g,rc);return rc;}
  g->goal=g->actual;g->requested=g->actual;/* Latch once on hold/start/reversal; discard old travel, never chase drift. */
 }
 if(direction!=g->direction)g->last_step=grip_now(g);
 g->direction=direction;return 0;
}
static inline int grip_resume(grip_controller *g){
 if(!g->active||!g->valid)return GP_NOT_ENABLED;
 int rc=grip_read(g);uint8_t on=0;
 if(!rc)rc=st_read(g->bus,6,40,1,&on,&g->error);
 if(!rc&&on!=1)rc=ST_FORMAT;
 if(rc){grip_freeze(g,rc);return rc;}
 /* Discard queued travel and hold measured position after explicit resume. */
 rc=st_set_goal(g->bus,6,(unsigned)g->actual,g->speed,5,&g->error);
 if(rc){grip_freeze(g,rc);return rc;}
 g->goal=g->actual;g->requested=g->actual;g->direction=0;g->frozen=0;g->reason=0;
 g->last_command=g->last_poll=g->last_step=grip_now(g);return 0;
}
static inline int grip_poll(grip_controller *g){
 if(!g->active)return 0;
 uint32_t now=grip_now(g);
 if((uint32_t)(now-g->last_poll)>=80U){
  g->last_poll=now;int rc=grip_read(g);uint8_t on=0;
  if(!rc)rc=st_read(g->bus,6,40,1,&on,&g->error);
  if(!rc&&on!=1)rc=ST_FORMAT;
  if(rc){grip_freeze(g,rc);return rc;}
 }
 if(g->frozen)return g->reason;
 if((uint32_t)(now-g->last_command)>500U){grip_freeze(g,GP_WATCHDOG);return GP_WATCHDOG;}
 uint32_t elapsed=now-g->last_step;if(!g->direction||elapsed<20U)return 0;
 g->last_step=now;double dt=elapsed>200U?.2:(double)elapsed/1000.;
 int endpoint=g->direction<0?g->closed:g->opened;
 double change=g->rate*dt;
 if(g->requested<endpoint)g->requested=fmin(endpoint,g->requested+change);
 else g->requested=fmax(endpoint,g->requested-change);
 int target=(int)lround(g->requested);
 int lo=g->closed<g->opened?g->closed:g->opened,hi=g->closed>g->opened?g->closed:g->opened;
 if(target<lo)target=lo;
 if(target>hi)target=hi;
 if(target==g->goal)return 0;
 if(!grip_bounds(g,target)){grip_freeze(g,GP_LIMIT);return GP_LIMIT;}
 int rc=st_set_goal(g->bus,6,(unsigned)target,g->speed,5,&g->error);
 if(rc){grip_freeze(g,rc);return rc;}
 g->goal=target;return 0;
}
#endif
