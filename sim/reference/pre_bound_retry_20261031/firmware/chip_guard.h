#ifndef CHIP_GUARD_H
#define CHIP_GUARD_H
#include <stdint.h>
typedef struct {int latched,valid,allowed,fan_pct;float celsius;uint32_t status;
 uint32_t hot_since;int hot_pending;} chip_guard;
static inline void chip_guard_update_at(chip_guard *g,uint32_t s,uint32_t now){
 unsigned raw=s&65535U;
 g->status=s;g->fan_pct=(int)((s>>16)&127U);
 g->valid=(s&(1U<<23)) && !(s&(1U<<26)) && raw>=32920U && raw<=51124U && (g->fan_pct==50 || g->fan_pct==100);
 g->celsius=(float)raw*503.975f/65536.0f-273.15f;
 if(!g->valid || (s&(1U<<27))){g->latched=1;g->hot_pending=0;}
 else if(g->celsius>60.0f){
  if(!g->hot_pending){g->hot_pending=1;g->hot_since=now;}
  else if((uint32_t)(now-g->hot_since)>1500U)g->latched=1;
 }else {g->hot_pending=0;if(g->celsius<55.0f)g->latched=0;}
 g->allowed=g->valid && !g->latched;
}
/* Compatibility helper for old read-only/native callers: no elapsed time. */
static inline void chip_guard_update(chip_guard *g,uint32_t s){chip_guard_update_at(g,s,0);}
#endif
