#include <assert.h>
#include <stdio.h>
#include "online_curve.h"
static void check_curve(online_curve *c,const curve_limits *l){
 for(int n=0;n<=2000;n++){
  c->elapsed=c->duration*n/2000.;double p[5],v[5],a[5];curve_state(c,p,v,a);
  double u=c->elapsed/c->duration;
  for(int i=0;i<5;i++){
   double j[3];for(int k=0;k<3;k++)j[k]=60*(c->b[i][k+3]-3*c->b[i][k+2]+3*c->b[i][k+1]-c->b[i][k])/(c->duration*c->duration*c->duration);
   assert(p[i]>=l->low[i]-1e-7&&p[i]<=l->high[i]+1e-7);
   assert(fabs(v[i])<=l->vmax[i]+1e-7&&fabs(a[i])<=l->amax[i]+1e-7);
   assert(fabs(curve_eval(j,2,u))<=l->jmax[i]+1e-6);
  }
 }
}
int main(void){
 curve_limits l;online_curve c;int p[5]={2000,2100,1900,2000,2100},goal[5]={2200,1700,2500,2800,1900};
 for(int i=0;i<5;i++){l.vmax[i]=i>=1&&i<=3?900:600;l.amax[i]=1200+150*i;l.jmax[i]=6000+500*i;l.low[i]=800;l.high[i]=3100;}
 curve_reset(&c,p);assert(curve_plan(&c,goal,&l));
 double oldp[5],oldv[5],olda[5];c.elapsed=c.duration*.32;curve_state(&c,oldp,oldv,olda);
 int next[5]={1800,2400,1700,1900,2300};assert(curve_plan(&c,next,&l));
 double np[5],nv[5],na[5];curve_state(&c,np,nv,na);
 for(int i=0;i<5;i++){assert(fabs(np[i]-oldp[i])<1e-8);assert(fabs(nv[i]-oldv[i])<1e-8);assert(fabs(na[i]-olda[i])<1e-7);}
 check_curve(&c,&l);curve_state(&c,np,nv,na);
 for(int i=0;i<5;i++){assert(fabs(np[i]-next[i])<1e-8);assert(nv[i]==0&&na[i]==0);}
 online_curve saved=c;next[3]=4000;assert(!curve_plan(&c,next,&l));assert(!memcmp(&saved,&c,sizeof c));
 unsigned seed=317;for(int n=0;n<200;n++){
  for(int i=0;i<5;i++){seed=1664525U*seed+1013904223U;goal[i]=1000+(int)(seed%1900);}
  assert(curve_plan(&c,goal,&l));check_curve(&c,&l);
 }
 puts("PASS continuous five-axis limits, synchronized endpoint, C2 replan, rejected goal transaction, 200 random motions");
}
