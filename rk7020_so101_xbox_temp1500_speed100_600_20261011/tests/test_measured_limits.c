#include <assert.h>
#include <stdio.h>
#include <math.h>
#include "../firmware/servo_mapping.h"
int main(void){
 const int lo[5]={1164,828,876,874,170},hi[5]={2964,3197,3073,3180,3966},zero[5]={2121,2156,1893,1986,2037},sign[5]={-1,1,1,1,-1};
 int c[5],back[5];float q[5]={0};unsigned n=0;
 for(int j=0;j<5;j++){
  const so101_joint_map *m=&so101_right_joints[j];assert(m->minimum==lo[j]&&m->maximum==hi[j]&&m->zero==zero[j]&&m->sign==sign[j]);
  for(int k=0;k<5;k++)c[k]=zero[k];
  c[j]=lo[j]-1;assert(so101_counts_to_q(c,q)==j+1);c[j]=hi[j]+1;assert(so101_counts_to_q(c,q)==j+1);
  for(int v=lo[j]+11;v<=hi[j]-11;v++){
   c[j]=v;assert(!so101_counts_to_q(c,q));assert(!so101_q_to_counts(q,back));assert(back[j]==v);n++;
  }
  for(int k=0;k<5;k++)q[k]=0;
  q[j]=m->q_min;assert(!so101_q_to_counts(q,back));assert(back[j]==(sign[j]>0?lo[j]+10:hi[j]-10));
  q[j]=m->q_max;assert(!so101_q_to_counts(q,back));assert(back[j]==(sign[j]>0?hi[j]-10:lo[j]+10));
  q[j]=m->q_min-.001f;assert(so101_q_to_counts(q,back)==j+1);q[j]=m->q_max+.001f;assert(so101_q_to_counts(q,back)==j+1);
 }
 for(int k=0;k<5;k++){q[k]=0;} q[3]=100*3.14159265358979323846/180;assert(!so101_q_to_counts(q,back));
 printf("PASS: %u encoder roundtrips, all five feedback boundaries and 10-count command margins, wrist100deg accepted, zeros/signs unchanged\n",n);return 0;
}

