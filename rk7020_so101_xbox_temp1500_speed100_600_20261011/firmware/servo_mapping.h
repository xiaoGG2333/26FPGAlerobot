/* Supplied R12254604 encoder baseline and 20261009 manual reference candidate for the corrected 135mm model.
 * Operator-aligned axis reference; absolute angular precision remains unverified. No motor/EEPROM operations.
 * Present_Position already includes the motor's Homing_Offset adjustment.
 */
#ifndef SO101_RIGHT_MAPPING_H
#define SO101_RIGHT_MAPPING_H
#include <math.h>
#include <stdint.h>
#ifndef SO101_PHYSICAL_ALIGNMENT_VERIFIED
#define SO101_PHYSICAL_ALIGNMENT_VERIFIED 0
#endif
#define SO101_COUNTS_PER_RAD 651.8986469044033f
typedef struct {int id,zero,sign,minimum,maximum;float q_min,q_max;} so101_joint_map;
/* Measured feedback endpoints; q bounds retain the existing 10-count command margin. */
static const so101_joint_map so101_right_joints[5]={
    {1, 2121, -1, 1164, 2964, -1.27780599631f, 1.45267980613f}, /* shoulder_pan */
    {2, 2156, 1, 828, 3197, -2.02178667843f, 1.58153419231f}, /* shoulder_lift */
    {3, 1893, 1, 876, 3073, -1.5447186534f, 1.79475752183f}, /* elbow_flex */
    {4, 1986, 1, 874, 3180, -1.69044682825f, 1.81623325286f}, /* wrist_flex */
    {5, 2037, -1, 170, 3966, -2.94370913195f, 2.8486023231f}, /* wrist_roll */
};
/* Pure preview conversion. Never clamp an unreachable target or partially
 * update the output. Returns 0=valid, 1..5=bad joint, -1=null argument. */
static inline int so101_q_to_counts(const float q[5],int counts[5])
{
    if(!q||!counts)return -1;
    int temporary[5];
    for(int i=0;i<5;++i){
        const so101_joint_map *j=&so101_right_joints[i];
        if(!isfinite(q[i])||q[i]<j->q_min||q[i]>j->q_max)return i+1;
        temporary[i]=(int)lroundf(j->zero+j->sign*q[i]*SO101_COUNTS_PER_RAD);
        if(temporary[i]<j->minimum||temporary[i]>j->maximum)return i+1;
    }
    for(int i=0;i<5;++i)counts[i]=temporary[i];
    return 0;
}
static inline int so101_counts_to_q(const int counts[5],float q[5])
{
    if(!q||!counts)return -1;
    for(int i=0;i<5;++i)
        if(counts[i]<so101_right_joints[i].minimum||counts[i]>so101_right_joints[i].maximum)return i+1;
    for(int i=0;i<5;++i)q[i]=so101_right_joints[i].sign*(counts[i]-so101_right_joints[i].zero)/SO101_COUNTS_PER_RAD;
    return 0;
}
#endif
