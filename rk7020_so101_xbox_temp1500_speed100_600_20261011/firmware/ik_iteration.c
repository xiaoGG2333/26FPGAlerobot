#include "ik_iteration.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846

void ik_fk(const float q[5], double T[16])
{
    /* Translate then rotate, same ET sequence as Python and current HLS IP. */
    const double tx[5]={.0612,.02943,.02798,.13504,.0593};
    const double tz[5]={.0598,.05504,.1127,.00519,.00996};
    double R[3][3]={{1,0,0},{0,1,0},{0,0,1}},p[3]={0,0,0};
    for(int joint=0;joint<5;++joint) {
        double c=cos((double)q[joint]),s=sin((double)q[joint]);
        double A[3][3]={{1,0,0},{0,1,0},{0,0,1}},next[3][3]={{0}};
        for(int i=0;i<3;++i)p[i]+=R[i][0]*tx[joint]+R[i][2]*tz[joint];
        if(joint==0) {A[0][0]=c;A[0][1]=-s;A[1][0]=s;A[1][1]=c;}
        else if(joint==4) {A[1][1]=c;A[1][2]=-s;A[2][1]=s;A[2][2]=c;}
        else {A[0][0]=c;A[0][2]=s;A[2][0]=-s;A[2][2]=c;}
        for(int i=0;i<3;++i)for(int j=0;j<3;++j)
            for(int k=0;k<3;++k)next[i][j]+=R[i][k]*A[k][j];
        memcpy(R,next,sizeof R);
    }
    memset(T,0,16*sizeof(double));T[15]=1;
    for(int i=0;i<3;++i) {
        for(int j=0;j<3;++j)T[4*i+j]=R[i][j];
        T[4*i+3]=p[i];
    }
}

void ik_error(const double current[16],const double target[16],float e[6])
{
    double R[3][3]={{0}},v[3],n,tr;
    for(int i=0;i<3;++i) {
        e[i]=(float)(target[4*i+3]-current[4*i+3]);
        for(int j=0;j<3;++j)for(int k=0;k<3;++k)
            R[i][j]+=target[4*i+k]*current[4*j+k];
    }
    v[0]=R[2][1]-R[1][2];v[1]=R[0][2]-R[2][0];v[2]=R[1][0]-R[0][1];
    n=sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);tr=R[0][0]+R[1][1]+R[2][2];
    /* Preserve original angle_axis near-zero/180-degree branch for parity.
       Arbitrary near-pi targets need separate validation, as in Python. */
    for(int i=0;i<3;++i)e[i+3]=(float)(n<1e-6 ?
        (tr>0?0:PI*.5*(R[i][i]+1)) : atan2(n,tr-1)*v[i]/n);
}

int ik_solve(const double target[16],const float initial[5],int max_steps,
             double tolerance,float damping,int sugihara,
             ik_step_fn step,ik_trace_fn trace,void *context,ik_result *out)
{
    double current[16];float e[6],delta[5];
    memset(out,0,sizeof *out);memcpy(out->q,initial,sizeof out->q);
    out->code=3;
    if(max_steps<0 || !(tolerance>0) || !isfinite(tolerance) ||
       !(damping>0) || !isfinite(damping) || !step)return out->code;
    for(int i=0;i<16;++i)if(!isfinite(target[i]))return out->code;
    for(int iteration=0;iteration<=max_steps;++iteration) {
        out->iterations=iteration;
        for(int i=0;i<5;++i)if(!isfinite(out->q[i]))return out->code=3;
        ik_fk(out->q,current);ik_error(current,target,e);
        double p2=0,r2=0;
        for(int i=0;i<3;++i) {p2+=(double)e[i]*e[i];r2+=(double)e[i+3]*e[i+3];}
        out->energy=.5*(p2+r2);out->position_error=sqrt(p2);out->rotation_error=sqrt(r2);
        if(!isfinite(out->energy))return out->code=3;
        float effective=sugihara?(float)(out->energy+damping):damping;
        if(trace)trace(context,iteration,out->q,out->energy,out->position_error,out->rotation_error,effective);
        if(out->energy<tolerance) {
            for(int i=0;i<5;++i) {
                double v=fmod((double)out->q[i]+PI,2*PI);
                if(v<0)v+=2*PI;
                out->q[i]=(float)(v-PI);
            }
            return out->code=0;
        }
        if(iteration==max_steps)return out->code=1;
        if(step(context,out->q,e,effective,delta)!=0)return out->code=2;
        for(int i=0;i<5;++i) {
            if(!isfinite(delta[i]))return out->code=3;
            out->q[i]+=delta[i];
        }
    }
    return out->code;
}
