#include <assert.h>
#include <stdio.h>
#include "../firmware/chip_guard.h"
static uint32_t sample(unsigned raw,int duty,unsigned flags){return raw|((unsigned)duty<<16)|(1U<<23)|flags;}
int main(void){chip_guard g={0};
 chip_guard_update(&g,0);assert(!g.allowed&&!g.valid&&g.latched);
 chip_guard_update(&g,sample(40000,50,0));assert(g.allowed&&g.valid&&g.fan_pct==50);
 chip_guard_update(&g,sample(45000,100,1U<<24));assert(g.allowed);
 chip_guard_update(&g,sample(45300,100,(1U<<24)|(1U<<25)));assert(!g.allowed&&g.latched);
 chip_guard_update(&g,sample(45000,100,1U<<24));assert(!g.allowed);
 chip_guard_update(&g,sample(43968,100,1U<<24));assert(g.allowed);
 chip_guard_update(&g,sample(40000,100,1U<<26));assert(!g.valid&&!g.allowed);
 chip_guard_update(&g,sample(40000,50,0));assert(g.allowed);
 chip_guard_update(&g,sample(40000,100,1U<<27));assert(!g.allowed);
 chip_guard_update(&g,sample(65535,100,0));assert(!g.valid&&!g.allowed);
 puts("PASS chip guard: invalid/startup, normal, full speed, hot latch, cooling recovery, stale, OT, range");return 0;}
