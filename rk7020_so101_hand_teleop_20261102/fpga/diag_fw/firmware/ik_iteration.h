#ifndef IK_ITERATION_H
#define IK_ITERATION_H
/* Port of create_so101_5dof, angle_axis and HWSolver.solve, mask = all ones.
   No servo control, line search or joint-limit enforcement. SI units. */
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
#endif
