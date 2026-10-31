#ifndef RK_ONLINE_CURVE_H
#define RK_ONLINE_CURVE_H
#include <math.h>
#include <string.h>
/* Ruckig-inspired state/limit interface; bounded quintic Bezier trajectories.
 * Not the Ruckig library, and not a time-optimal seven-segment algorithm.
 * Bernstein derivative bounds certify the COMPLETE continuous trajectory.
 * Counts, seconds. All five axes share one duration. */
typedef struct {double vmax[5],amax[5],jmax[5],low[5],high[5];} curve_limits;
typedef struct {double b[5][6],duration,elapsed;int valid;} online_curve;
static inline int curve_bound(const double *b,int n,double low,double high,int depth){
 int inside=1;for(int k=0;k<=n;k++){if(!isfinite(b[k]))return 0;if(b[k]<low||b[k]>high)inside=0;}
 if(inside)return 1;
 if(!depth||b[0]<low||b[0]>high||b[n]<low||b[n]>high)return 0;
 double temp[6],left[6],right[6];for(int k=0;k<=n;k++)temp[k]=b[k];
 left[0]=temp[0];right[n]=temp[n];
 for(int k=1;k<=n;k++){for(int j=0;j<=n-k;j++)temp[j]=.5*(temp[j]+temp[j+1]);left[k]=temp[0];right[n-k]=temp[n-k];}
 return curve_bound(left,n,low,high,depth-1)&&curve_bound(right,n,low,high,depth-1);
}
static inline double curve_eval(const double *b,int degree,double u){
 double x[6];for(int i=0;i<=degree;i++)x[i]=b[i];
 for(int n=degree;n>0;n--)for(int i=0;i<n;i++)x[i]+=(x[i+1]-x[i])*u;
 return x[0];
}
static inline void curve_state(const online_curve *c,double p[5],double v[5],double a[5]){
 double u=c->duration>0?fmin(1.,c->elapsed/c->duration):1.;
 for(int i=0;i<5;i++){
  double db[5],ddb[4];p[i]=curve_eval(c->b[i],5,u);
  for(int k=0;k<5;k++)db[k]=5*(c->b[i][k+1]-c->b[i][k])/c->duration;
  for(int k=0;k<4;k++)ddb[k]=4*(db[k+1]-db[k])/c->duration;
  v[i]=u>=1?0:curve_eval(db,4,u);a[i]=u>=1?0:curve_eval(ddb,3,u);
 }
}
static inline void curve_reset(online_curve *c,const int p[5]){
 memset(c,0,sizeof *c);c->duration=.02;c->elapsed=.02;c->valid=1;
 for(int i=0;i<5;i++)for(int k=0;k<6;k++)c->b[i][k]=p[i];
}
static inline int curve_plan(online_curve *c,const int goal[5],const curve_limits *l){
 double p[5],v[5],a[5];curve_state(c,p,v,a);
 for(int i=0;i<5;i++)if(!isfinite(p[i])||!isfinite(v[i])||!isfinite(a[i])||
   !isfinite(l->vmax[i])||!isfinite(l->amax[i])||!isfinite(l->jmax[i])||
   l->vmax[i]<=0||l->amax[i]<=0||l->jmax[i]<=0||goal[i]<l->low[i]||goal[i]>l->high[i])return 0;
 /* Search common duration; the derivative convex hull is conservative.
    Reject an infeasible reversal near a boundary without disturbing old plan. */
 for(double T=.04;T<=60;T*=1.08){
  online_curve trial;memset(&trial,0,sizeof trial);trial.duration=T;trial.valid=1;
  int good=1;
  for(int i=0;i<5&&good;i++){
   double *b=trial.b[i],d[5],dd[4],ddd[3];
   b[0]=p[i];b[1]=p[i]+v[i]*T/5;
   b[2]=p[i]+2*v[i]*T/5+a[i]*T*T/20;
   b[3]=b[4]=b[5]=goal[i];
   for(int k=0;k<5;k++)d[k]=5*(b[k+1]-b[k])/T;
   for(int k=0;k<4;k++)dd[k]=4*(d[k+1]-d[k])/T;
   for(int k=0;k<3;k++)ddd[k]=3*(dd[k+1]-dd[k])/T;
   good=curve_bound(b,5,l->low[i],l->high[i],5)&&curve_bound(d,4,-l->vmax[i],l->vmax[i],5)&&
        curve_bound(dd,3,-l->amax[i],l->amax[i],5)&&curve_bound(ddd,2,-l->jmax[i],l->jmax[i],5);
  }
  if(good){*c=trial;return 1;}
 }
 return 0;
}
#endif
