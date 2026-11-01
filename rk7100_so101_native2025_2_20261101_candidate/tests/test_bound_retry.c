#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "ik_iteration.h"
#include "servo_mapping.h"
static int calls,mode;static float starts[3][5];
static int fake(void *ctx,const float q[5],const float e[6],float lambda,float out[5]){
 (void)ctx;(void)e;(void)lambda;
 if(calls==0||calls==5||calls==10)memcpy(starts[calls/5],q,5*sizeof(float));
 calls++;if(mode==1)return 1;
 for(int i=0;i<5;i++)out[i]=0;
 if(mode==2)out[0]=1e-6f;
 return 0;
}
int main(void){
 float seed[5]={0,-.2f,.3f,.1f,0};double T[16],xyz[3];ik_result out;ik_fk(seed,T);
 xyz[0]=T[3];xyz[1]=T[7];xyz[2]=T[11];
 assert(!ik_solve_position_roll_retry(xyz,.2f,seed,200,1e-8,.01f,fake,0,&out)&&calls==0);
 for(int i=0;i<4;i++)assert(out.q[i]==seed[i]);
 assert(ik_solve_position_roll_retry(xyz,4,seed,200,1e-8,.01f,fake,0,&out)==4&&calls==0);
 xyz[0]+=1;mode=1;
 assert(ik_solve_position_roll_retry(xyz,0,seed,200,1e-8,.01f,fake,0,&out)==2&&calls==1);
 calls=0;mode=0;
 assert(ik_solve_position_roll_retry(xyz,0,seed,200,1e-8,.01f,fake,0,&out)==1&&calls==15);
 assert(fabsf(starts[0][2]-seed[2])<1e-7f);
 assert(starts[1][2]<seed[2]&&starts[2][2]>seed[2]);
 assert(starts[1][3]>seed[3]&&starts[2][3]<seed[3]);
 for(int a=0;a<3;a++)for(int i=0;i<4;i++)assert(starts[a][i]>=so101_right_joints[i].q_min&&starts[a][i]<=so101_right_joints[i].q_max);
 memset(seed,0,sizeof seed);float target[5]={1,0,0,0,0};ik_fk(target,T);xyz[0]=T[3];xyz[1]=T[7];xyz[2]=T[11];calls=0;mode=2;
 assert(ik_solve_position_roll_retry(xyz,0,seed,1000,1e-20,.01f,fake,0,&out)==1&&calls==600);
 puts("PASS zero-step success, invalid ID5 no calls, hardware fault terminal, exactly two distinct legal seeds, shared600-call budget");
}
