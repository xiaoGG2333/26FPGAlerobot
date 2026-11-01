#include <cmath>
#include <cstdio>
extern "C" void geometry_preview_135mm(const float[5],const float[5],float[3],float[3][3],float[6][5]);
extern "C" void xf_solver_lm_so101(float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,float,volatile float*,volatile float*,volatile float*,volatile float*,volatile float*,volatile int*,volatile int*);
static void ref_fk(const float q[5],double p[3],double R[3][3]) {
    const double x[5]={.0612,.02943,.02798,.13504,.0593};
    const double z[5]={.0598,.05504,.1127,.00519,.00996};
    for(int i=0;i<3;i++){p[i]=0;for(int j=0;j<3;j++)R[i][j]=i==j;}
    for(int k=0;k<5;k++){
        for(int i=0;i<3;i++)p[i]+=R[i][0]*x[k]+R[i][2]*z[k];
        double a[3][3]={{1,0,0},{0,1,0},{0,0,1}},b[3][3]={};
        double c=std::cos(double(q[k])),s=std::sin(double(q[k]));
        int u=k==0?0:k==4?1:0,v=k==0?1:k==4?2:2;
        a[u][u]=a[v][v]=c;a[u][v]=(k>=1&&k<=3)?s:-s;a[v][u]=-a[u][v];
        for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int t=0;t<3;t++)b[i][j]+=R[i][t]*a[t][j];
        for(int i=0;i<3;i++)for(int j=0;j<3;j++)R[i][j]=b[i][j];
    }
}
static int checks=0;
static bool near(double a,double b,double tol,const char* name){checks++;if(std::fabs(a-b)>tol){std::printf("FAIL %s %.9g vs %.9g\n",name,a,b);return false;}return true;}
static bool converge(const float target[5],const float initial[5]) {
    double goal_p[3],goal_R[3][3];ref_fk(target,goal_p,goal_R);
    float q[5];for(int i=0;i<5;i++)q[i]=initial[i];
    for(int it=0;it<=500;it++){
        double p[3],R[3][3],D[3][3]={};ref_fk(q,p,R);float e[6];
        for(int i=0;i<3;i++)e[i]=float(goal_p[i]-p[i]);
        for(int i=0;i<3;i++)for(int j=0;j<3;j++)for(int k=0;k<3;k++)D[i][j]+=goal_R[i][k]*R[j][k];
        double cosine=(D[0][0]+D[1][1]+D[2][2]-1)*.5;
        if(cosine>1)cosine=1;if(cosine< -1)cosine=-1;
        double angle=std::acos(cosine),factor=angle<1e-7?.5:angle/(2*std::sin(angle));
        e[3]=float(factor*(D[2][1]-D[1][2]));e[4]=float(factor*(D[0][2]-D[2][0]));e[5]=float(factor*(D[1][0]-D[0][1]));
        double pe=0,re=0;for(int i=0;i<3;i++){pe+=double(e[i])*e[i];re+=double(e[i+3])*e[i+3];}
        if(std::sqrt(pe)<2e-5&&std::sqrt(re)<2e-5){std::printf("135MM HLS FULL ITERATION PASS iter=%d pos=%.9g rot=%.9g\n",it,std::sqrt(pe),std::sqrt(re));return true;}
        float s[5],c[5],d[5];int status=-1,state=0;
        for(int i=0;i<5;i++){s[i]=std::sin(q[i]);c[i]=std::cos(q[i]);}
        xf_solver_lm_so101(s[0],s[1],s[2],s[3],s[4],c[0],c[1],c[2],c[3],c[4],e[0],e[1],e[2],e[3],e[4],e[5],.01f,&d[0],&d[1],&d[2],&d[3],&d[4],&status,&state);
        if(status!=0||(state&2)!=2)return false;
        for(int i=0;i<5;i++){if(!std::isfinite(d[i]))return false;q[i]+=d[i];}
    }
    std::printf("FAIL 135MM HLS full iteration did not converge\n");return false;
}

// Production PS damping acceptance exercised against the actual HLS top.
#include "../firmware/ik_iteration.c"
static double production_last_energy;
static int production_calls;
static void production_trace(void *,int,const float[5],double E,double,double,float){
    if(E>production_last_energy+1e-12){std::printf("FAIL adaptive accepted increasing energy\n");production_last_energy=-1;}
    else production_last_energy=E;
}
static int production_hw(void *,const float q[5],const float e[6],float damping,float d[5]){
    float ss[5],cc[5];for(int i=0;i<5;i++){ss[i]=sinf(q[i]);cc[i]=cosf(q[i]);}
    int status=-1,state=0;production_calls++;
    xf_solver_lm_so101(ss[0],ss[1],ss[2],ss[3],ss[4],cc[0],cc[1],cc[2],cc[3],cc[4],e[0],e[1],e[2],e[3],e[4],e[5],damping,&d[0],&d[1],&d[2],&d[3],&d[4],&status,&state);
    return status;
}
static bool production_regression(){
    const float goals[3][5]={{.2f,-.3f,.4f,-.2f,.15f},{-.25f,.2f,-.35f,.15f,-.2f},{.7f,-.5f,.8f,-.4f,.6f}};
    const float initial[5]={0,0,0,0,0};
    for(int n=0;n<3;n++){
        double T[16];ik_fk(goals[n],T);ik_result result;production_calls=0;production_last_energy=1e100;
        int code=ik_solve_adaptive(T,initial,200,1e-8,.01f,1,production_hw,production_trace,0,&result);
        if(code||result.energy>=1e-8||production_last_energy<0){std::printf("FAIL production adaptive case=%d code=%d E=%.9g\n",n,code,result.energy);return false;}
        std::printf("PRODUCTION ADAPTIVE PASS case=%d iterations=%d calls=%d E=%.9g\n",n,result.iterations,production_calls,result.energy);
    }
    return true;
}

static bool position_roll_regression(){
    // Exercise task-space Jacobian against an independent finite-difference
    // normal-equation solve; unused angular error slots must have no effect.
    float q[5]={.3f,-.4f,.5f,-.2f,.7f};double base_p[3],base_R[3][3];
    ref_fk(q,base_p,base_R);double J[6][5]={},A[5][6]={};
    float e[6]={.003f,-.002f,.004f,.025f,5.f,-7.f};
    for(int j=0;j<5;j++){
        float plus[5],minus[5];for(int k=0;k<5;k++)plus[k]=minus[k]=q[k];
        plus[j]+=.001f;minus[j]-=.001f;double P[3],M[3],R[3][3];
        ref_fk(plus,P,R);ref_fk(minus,M,R);double h=double(plus[j])-minus[j];
        for(int k=0;k<3;k++)J[k][j]=(P[k]-M[k])/h;
        J[3][j]=j==4?1.:0.;
    }
    for(int i=0;i<5;i++){
        for(int j=0;j<5;j++)for(int k=0;k<4;k++)A[i][j]+=J[k][i]*J[k][j];
        A[i][i]+=.01;for(int k=0;k<4;k++)A[i][5]+=J[k][i]*e[k];
    }
    for(int i=0;i<5;i++){
        double v=A[i][i];for(int j=i;j<6;j++)A[i][j]/=v;
        for(int k=0;k<5;k++)if(k!=i){double f=A[k][i];for(int j=i;j<6;j++)A[k][j]-=f*A[i][j];}
    }
    float d[5];if(production_hw(0,q,e,-.01f,d))return false;
    for(int i=0;i<5;i++)if(!near(d[i],A[i][5],3e-6,"position roll task normal equation"))return false;
    if(!near(d[4],.025/1.01,1e-7,"direct ID5 task"))return false;
    const double xyz[2][3]={{0.169340303547584,0.28936456848168,0.17145964303696},{0.163856602771086,0.0561544363681065,0.302322726533606}};
    const float seeds[2][5]={{1.20417492f,1.02623315f,-1.23638852f,1.20724288f,-0.566038911f},{0.492407833f,-0.539961237f,-0.365087428f,1.80856335f,0.190213618f}};
    const float rolls[2]={-0.571553526f,0.204574255f};
    for(int n=0;n<2;n++){
        ik_result out;production_calls=0;
        int rc=ik_solve_position_roll(xyz[n],rolls[n],seeds[n],200,1e-8,.01f,production_hw,0,&out);
        if(rc||out.position_error>=sqrt(2e-8)||out.rotation_error>1e-7)return false;
        for(int i=0;i<5;i++)if(out.q[i]<so101_right_joints[i].q_min||out.q[i]>so101_right_joints[i].q_max)return false;
        double oldT[16],newT[16];ik_fk(seeds[n],oldT);ik_fk(out.q,newT);
        double oldpitch=atan2(-oldT[8],hypot(oldT[0],oldT[4])),newpitch=atan2(-newT[8],hypot(newT[0],newT[4]));
        printf("POSITION ROLL PASS recorded_case=%d iterations=%d pos=%.9g pitch_change=%.9g ID5_error=%.9g\n",n,out.iterations,out.position_error,newpitch-oldpitch,out.rotation_error);
        // Same XYZ, isolated ID5 commands in both directions: no arm-joint motion.
        for(int direction=-1;direction<=1;direction+=2){
            float targetroll=out.q[4]+direction*.1f;ik_result wrist;
            double T[16];ik_fk(out.q,T);double fixed[3]={T[3],T[7],T[11]};
            if(ik_solve_position_roll(fixed,targetroll,out.q,200,1e-8,.01f,production_hw,0,&wrist))return false;
            for(int i=0;i<4;i++)if(wrist.q[i]!=out.q[i])return false;
            if(fabs(wrist.q[4]-targetroll)>1e-7)return false;
        }
    }
    ik_result invalid;production_calls=0;
    if(ik_solve_position_roll(xyz[0],4.f,seeds[0],200,1e-8,.01f,production_hw,0,&invalid)!=4||production_calls)return false;
    double far[3]={2,0,0};if(!ik_solve_position_roll(far,rolls[0],seeds[0],40,1e-8,.01f,production_hw,0,&invalid))return false;
    puts("POSITION ROLL TASK REGRESSION PASS: free pitch, bounded warm start, direct wrist +/- and unreachable rejection");
    return true;
}


static bool bound_mask_regression(){
 const float q[5]={.2f,-.4f,.5f,.1f,.3f};
 for(unsigned mask=0;mask<16;mask++){
  float e[6]={.01f,-.02f,.03f,0,0,(float)mask},d[5];
  double J[3][5],A[5][6]={};
  for(int j=0;j<5;j++){
   float qp[5],qm[5];for(int i=0;i<5;i++)qp[i]=qm[i]=q[i];qp[j]+=.001f;qm[j]-=.001f;
   double P[3],M[3],R[3][3];ref_fk(qp,P,R);ref_fk(qm,M,R);
   for(int k=0;k<3;k++)J[k][j]=(j==4||(mask&(1U<<j)))?0:(P[k]-M[k])/(double(qp[j])-qm[j]);
  }
  for(int i=0;i<5;i++){
   for(int j=0;j<5;j++)for(int k=0;k<3;k++)A[i][j]+=J[k][i]*J[k][j];
   A[i][i]+=.01+(i==4?1:0);
   for(int k=0;k<3;k++)A[i][5]+=J[k][i]*e[k];
  }
  for(int i=0;i<5;i++){
   double v=A[i][i];for(int j=i;j<6;j++)A[i][j]/=v;
   for(int k=0;k<5;k++)if(k!=i){double f=A[k][i];for(int j=i;j<6;j++)A[k][j]-=f*A[i][j];}
  }
  if(production_hw(0,q,e,-.01f,d))return false;
  for(int i=0;i<5;i++)if(!near(d[i],A[i][5],3e-6,"active mask reduced LM"))return false;
  for(int i=0;i<4;i++)if(mask&(1U<<i))if(!near(d[i],0,1e-8,"frozen direction zero"))return false;
 }
 puts("ACTIVE MASK PASS all16 constrained normal-equation cases");return true;
}


static bool retry_hardware_regression(){
 {double xyz[3]={0.18591956486130806,0.14313863188247419,0.025836083871495656};float seed[5]={0.691883031f,-2.02178668f,-0.0777612369f,0.451941409f,-1.93406462f};ik_result out;production_calls=0;
 int rc=ik_solve_position_roll_retry(xyz,-1.55499835f,seed,200,1e-8,.01f,production_hw,0,&out);
 printf("RETRY HW case=0 rc=%d iter=%d calls=%d error=%.9g\n",rc,out.iterations,production_calls,out.position_error);
 if(rc||out.position_error>=sqrt(2e-8)||production_calls>600)return false;
 }
 {double xyz[3]={0.10252188693030187,0.089838827774129257,0.095112058557260293};float seed[5]={1.37017067f,-2.02178668f,1.59122956f,1.29888367f,-0.298403683f};ik_result out;production_calls=0;
 int rc=ik_solve_position_roll_retry(xyz,0.37824076f,seed,200,1e-8,.01f,production_hw,0,&out);
 printf("RETRY HW case=1 rc=%d iter=%d calls=%d error=%.9g\n",rc,out.iterations,production_calls,out.position_error);
 if(rc||out.position_error>=sqrt(2e-8)||production_calls>600)return false;
 }
 return true;
}

int main(){
    bool ok=true;
    const float cases[4][5]={{0,0,0,0,0},{.1f,-.2f,.3f,-.1f,.15f},{-.25f,.2f,-.35f,.15f,-.2f},{.8f,-.6f,1.f,-.7f,.9f}};
    for(int n=0;n<4;n++){
        float s[5],c[5],p[3],R[3][3],J[6][5];double rp[3],rR[3][3];
        for(int j=0;j<5;j++){s[j]=std::sin(cases[n][j]);c[j]=std::cos(cases[n][j]);}
        geometry_preview_135mm(s,c,p,R,J);
        ref_fk(cases[n],rp,rR);
        for(int i=0;i<3;i++){ok=near(p[i],rp[i],2e-7,"FK position")&&ok;for(int j=0;j<3;j++)ok=near(R[i][j],rR[i][j],3e-7,"FK rotation")&&ok;}
        for(int j=0;j<5;j++){
            float plus[5],minus[5];for(int i=0;i<5;i++)plus[i]=minus[i]=cases[n][i];
            plus[j]+=.001f;minus[j]-=.001f;double pp[3],pm[3],ap[3][3],am[3][3],W[3][3]={};
            ref_fk(plus,pp,ap);ref_fk(minus,pm,am);
            double h=double(plus[j])-double(minus[j]);
            for(int i=0;i<3;i++)ok=near(J[i][j],(pp[i]-pm[i])/h,2e-6,"Jacobian position")&&ok;
            for(int a=0;a<3;a++)for(int b=0;b<3;b++)for(int t=0;t<3;t++)W[a][b]+=(ap[a][t]-am[a][t])/h*rR[b][t];
            ok=near(J[3][j],W[2][1],2e-6,"Jacobian angular x")&&ok;
            ok=near(J[4][j],W[0][2],2e-6,"Jacobian angular y")&&ok;
            ok=near(J[5][j],W[1][0],2e-6,"Jacobian angular z")&&ok;
        }
    }
    float s[5],c[5],d[5];for(int i=0;i<5;i++){s[i]=std::sin(cases[1][i]);c[i]=std::cos(cases[1][i]);}int status=-1,state=0;
    xf_solver_lm_so101(s[0],s[1],s[2],s[3],s[4],c[0],c[1],c[2],c[3],c[4],.02f,.015f,.01f,0,0,0,.01f,&d[0],&d[1],&d[2],&d[3],&d[4],&status,&state);
    const double expected[5]={.0027827176995747576,.06582481556135059,-.08806046114004273,.021720729533915083,0};
    for(int i=0;i<5;i++)ok=near(d[i],expected[i],2e-6,"135mm LM step")&&ok;
    ok=near(status,0,0,"status")&&ok;ok=near(state&2,2,0,"done")&&ok;
    xf_solver_lm_so101(s[0],s[1],s[2],s[3],s[4],c[0],c[1],c[2],c[3],c[4],0,0,0,0,0,0,.01f,&d[0],&d[1],&d[2],&d[3],&d[4],&status,&state);
    for(int i=0;i<5;i++)ok=near(d[i],0,1e-8,"zero error")&&ok;
    std::printf("135MM GEOMETRY REGRESSION %s: %d checks\n",ok?"PASS":"FAIL",checks);
    const float target0[5]={.2f,-.3f,.4f,-.2f,.15f},initial0[5]={.1f,-.2f,.3f,-.1f,0};
    ok=converge(target0,initial0)&&ok;
    ok=converge(cases[2],cases[0])&&ok;
    ok=production_regression()&&ok;
    ok=position_roll_regression()&&ok;
    ok=bound_mask_regression()&&ok;
    ok=retry_hardware_regression()&&ok;
    return ok?0:1;
}
