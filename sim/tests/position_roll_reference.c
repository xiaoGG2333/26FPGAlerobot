#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "ik_iteration.h"
static int calls;
int numerical_step(void *ctx,const float q[5],const float e[6],float damping,float out[5]){
 (void)ctx;calls++;double T[16],J[6][5],a[5][6]={0};ik_fk(q,T);
 for(int j=0;j<5;j++){
  float qp[5],qm[5];for(int k=0;k<5;k++)qp[k]=qm[k]=q[k];qp[j]+=.001f;qm[j]-=.001f;
  double P[16],M[16];ik_fk(qp,P);ik_fk(qm,M);double h=(double)qp[j]-qm[j],W[3][3]={0};
  for(int k=0;k<3;k++)J[k][j]=(P[4*k+3]-M[4*k+3])/h;
  for(int u=0;u<3;u++)for(int v=0;v<3;v++)for(int k=0;k<3;k++)W[u][v]+=(P[4*u+k]-M[4*u+k])/h*T[4*v+k];
  J[3][j]=W[2][1];J[4][j]=W[0][2];J[5][j]=W[1][0];
 }
 if(damping<0){damping=-damping;for(int j=0;j<5;j++){J[3][j]=j==4?1:0;J[4][j]=J[5][j]=0;}unsigned mask=(unsigned)e[5];for(int j=0;j<4;j++)if(mask&(1U<<j))for(int k=0;k<6;k++)J[k][j]=0;}
 for(int i=0;i<5;i++){for(int j=0;j<5;j++){for(int k=0;k<6;k++)a[i][j]+=J[k][i]*J[k][j];}a[i][i]+=damping;for(int k=0;k<6;k++)a[i][5]+=J[k][i]*e[k];}
 for(int i=0;i<5;i++){
  double pivot=a[i][i];if(!(pivot>0))return 1;
  for(int j=i;j<6;j++)a[i][j]/=pivot;
  for(int k=0;k<5;k++)if(k!=i){double f=a[k][i];for(int j=i;j<6;j++)a[k][j]-=f*a[i][j];}
 }
 for(int i=0;i<5;i++)out[i]=(float)a[i][5];
 return 0;
}
