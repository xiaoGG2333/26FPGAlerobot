#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "ik_iteration.h"
static int calls;
static int numerical_step(void *ctx,const float q[5],const float e[6],float damping,float out[5]){
 (void)ctx;calls++;double T[16],J[6][5],a[5][6]={0};ik_fk(q,T);
 for(int j=0;j<5;j++){
  float qp[5],qm[5];for(int k=0;k<5;k++)qp[k]=qm[k]=q[k];qp[j]+=.001f;qm[j]-=.001f;
  double P[16],M[16];ik_fk(qp,P);ik_fk(qm,M);double h=(double)qp[j]-qm[j],W[3][3]={0};
  for(int k=0;k<3;k++)J[k][j]=(P[4*k+3]-M[4*k+3])/h;
  for(int u=0;u<3;u++)for(int v=0;v<3;v++)for(int k=0;k<3;k++)W[u][v]+=(P[4*u+k]-M[4*u+k])/h*T[4*v+k];
  J[3][j]=W[2][1];J[4][j]=W[0][2];J[5][j]=W[1][0];
 }
 for(int i=0;i<5;i++){for(int j=0;j<5;j++){for(int k=0;k<6;k++)a[i][j]+=J[k][i]*J[k][j];}a[i][i]+=damping;for(int k=0;k<6;k++)a[i][5]+=J[k][i]*e[k];}
 for(int i=0;i<5;i++){
  double pivot=a[i][i];if(!(pivot>0))return 1;
  for(int j=i;j<6;j++)a[i][j]/=pivot;
  for(int k=0;k<5;k++)if(k!=i){double f=a[k][i];for(int j=i;j<6;j++)a[k][j]-=f*a[i][j];}
 }
 for(int i=0;i<5;i++)out[i]=(float)a[i][5];
 return 0;
}
static double last;
static void trace(void *ctx,int n,const float q[5],double E,double p,double rot,float d){
 (void)ctx;(void)n;(void)q;(void)p;(void)rot;(void)d;assert(E<=last+1e-12);last=E;
}
static int uphill(void *ctx,const float q[5],const float e[6],float d,float out[5]){
 (void)ctx;(void)q;(void)e;(void)d;calls++;for(int i=0;i<5;i++)out[i]=2.f;return 0;
}
int main(void){
 float zero[5]={0},targetq[5]={.2f,-.3f,.4f,-.2f,.15f};double T[16];ik_result res;ik_fk(targetq,T);
 last=1e99;assert(!ik_solve_adaptive(T,zero,200,1e-8,.01f,1,numerical_step,trace,0,&res));
 assert(res.energy<1e-8);
 calls=0;last=1e99;int rc=ik_solve_adaptive(T,zero,20,1e-8,.01f,1,uphill,trace,0,&res);
 assert(rc==1&&calls==5);for(int i=0;i<5;i++)assert(res.q[i]==0);
 ik_fk(zero,T);calls=0;assert(!ik_solve_adaptive(T,zero,20,1e-8,.01f,1,numerical_step,trace,0,&res)&&calls==0);
 puts("PASS adaptive damping convergence, monotonic acceptance, uphill retry rejection/unchanged seed, zero-error no-step");
}
