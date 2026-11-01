#ifndef IK_ITERATION_H
#define IK_ITERATION_H
/* Port of create_so101_5dof, angle_axis and HWSolver.solve, mask = all ones.
   Legacy full-pose entry is preserved; position task has bounded LM. SI units. */
typedef int (*ik_step_fn)(void *, const float[5], const float[6], float, float[5]);
typedef void (*ik_trace_fn)(void *, int, const float[5], double, double, double, float);
typedef struct {
    int code; /* 0 converged, 1 max iterations, 2 hardware error, 3 numerical error */
    int iterations;
    float q[5];
    double energy, position_error, rotation_error;
} ik_result;
void ik_fk(const float q[5], double T[16]);
void ik_error(const double current[16], const double target[16], float e[6]);
int ik_solve(const double target[16], const float initial[5], int max_steps,
             double tolerance, float damping, int sugihara,
             ik_step_fn step, ik_trace_fn trace, void *context, ik_result *result);
int ik_solve_adaptive(const double[16],const float[5],int,double,float,int,ik_step_fn,ik_trace_fn,void *,ik_result *);
/* XYZ position priority; ID5 is a joint-angle command, pitch/yaw are free.
 * Matching PL negative-lambda task mode is required. Rotation error reports
 * ID5 task error, not full end-effector orientation error. */
int ik_solve_position_roll(const double[3],float,const float[5],int,double,float,ik_step_fn,void *,ik_result *);
/* Bounded position task with up to two numerical-only nearby seed retries. */
int ik_solve_position_roll_retry(const double[3],float,const float[5],int,double,float,ik_step_fn,void *,ik_result *);
#endif
