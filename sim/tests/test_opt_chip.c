#include <assert.h>
#include <stdio.h>
#include "chip_guard.h"
static uint32_t s(double t){unsigned raw=(unsigned)((t+273.15)*65536/503.975+.5);return raw|(100U<<16)|(1U<<23);}
int main(void){chip_guard g={0};
 chip_guard_update_at(&g,s(32),0);assert(g.allowed);
 chip_guard_update_at(&g,s(61),100);chip_guard_update_at(&g,s(61),1600);assert(g.allowed);
 chip_guard_update_at(&g,s(59),1601);assert(!g.hot_pending&&g.allowed);
 chip_guard_update_at(&g,s(61),2000);chip_guard_update_at(&g,s(61),3500);assert(g.allowed);
 chip_guard_update_at(&g,s(61),3501);assert(!g.allowed&&g.latched);
 chip_guard_update_at(&g,s(57),3600);assert(!g.allowed);
 chip_guard_update_at(&g,s(54),3700);assert(g.allowed);
 chip_guard_update_at(&g,0,3800);assert(!g.allowed&&!g.valid);
 chip_guard_update_at(&g,s(32),3900);assert(g.allowed);
 chip_guard_update_at(&g,s(32)|(1U<<27),4000);assert(!g.allowed);
 chip_guard_update_at(&g,s(54),4100);assert(g.allowed);
 chip_guard_update_at(&g,s(61),0xffffff00U);chip_guard_update_at(&g,s(61),0xffffff00U+1501U);assert(!g.allowed);
 puts("PASS chip >60C strict >1500ms, transient filter, <55C recovery, sensor/OT immediate fault, timer wrap");
}
