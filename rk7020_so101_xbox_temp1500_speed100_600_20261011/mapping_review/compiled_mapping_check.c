
#include <assert.h>
#include <stdio.h>
#include "servo_mapping.h"
int main(void) {
  float q[5]={0}, restored[5]; int counts[5], n_encoder=0, n_angles=0; float max_error=0;
  for(int i=0;i<5;i++) {
    for(int n=so101_right_joints[i].minimum;n<=so101_right_joints[i].maximum;n++) {
      for(int k=0;k<5;k++) counts[k]=so101_right_joints[k].zero;
      counts[i]=n; assert(!so101_counts_to_q(counts,q));
      if(q[i]<so101_right_joints[i].q_min || q[i]>so101_right_joints[i].q_max) continue;
      assert(!so101_q_to_counts(q,counts)); assert(counts[i]==n); n_encoder++;
    }
    for(int n=0;n<=10000;n++) {
      for(int k=0;k<5;k++) q[k]=0;
      q[i]=so101_right_joints[i].q_min+(so101_right_joints[i].q_max-so101_right_joints[i].q_min)*n/10000.0f;
      /* Arithmetic at the final endpoint may exceed the stored float by one ulp. */
      if(n==10000) q[i]=so101_right_joints[i].q_max;
      assert(!so101_q_to_counts(q,counts)); assert(!so101_counts_to_q(counts,restored));
      float e=fabsf(q[i]-restored[i]); assert(e < .502f/SO101_COUNTS_PER_RAD);
      if(e>max_error) max_error=e; n_angles++;
    }
  }
  for(int i=0;i<5;i++) q[i]=0;
  assert(!so101_q_to_counts(q,counts)); int saved[5]; for(int i=0;i<5;i++) saved[i]=counts[i];
  q[4]=NAN; assert(so101_q_to_counts(q,counts)==5);
  for(int i=0;i<5;i++) assert(counts[i]==saved[i]);
  printf("%d %d %.9g\n",n_encoder,n_angles,max_error);
}
