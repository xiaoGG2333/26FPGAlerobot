/* RK7020 bare-metal IK diagnostic; preserves working UART0 initialization. */
#include "xparameters.h"
#include "xuartps.h"
#include "xstatus.h"
#include "sleep.h"

#define RK_UART0_BASE 0xE0000000U
static XUartPs uart;
volatile int uart_test_stage = 0;
volatile unsigned int uart_test_heartbeat = 0;
volatile unsigned int uart_test_tx_bytes = 0;

static void fail(int stage)
{
    uart_test_stage = stage;
    for (;;) { /* Pause CPU in debugger and inspect uart_test_stage. */ }
}

static void send_text(const char *text)
{
    while (*text) {
        unsigned int spins = 0;
        u8 ch = (u8)*text;
        while (XUartPs_Send(&uart, &ch, 1) != 1) {
            if (++spins >= 1000000U) fail(-5);
        }
        ++uart_test_tx_bytes;
        ++text;
    }
}


#include "xil_io.h"
#include <string.h>
#include <stdio.h>
#define REG_CTRL 0x0U
#define REG_LAMBDA 0x90U
#define REG_STATUS 0xE8U
#define REG_STATE_I 0xF8U
static const u32 sin_offset[5] = {0x10U,0x18U,0x20U,0x28U,0x30U};
static const u32 cos_offset[5] = {0x38U,0x40U,0x48U,0x50U,0x58U};
static const u32 e_offset[6] = {0x60U,0x68U,0x70U,0x78U,0x80U,0x88U};
static const u32 d_offset[5] = {0x98U,0xa8U,0xb8U,0xc8U,0xd8U};
static const float sin_q[5] = {0.0998334166f,-0.1986693308f,0.2955202067f,-0.0998334166f,0.1494381325f};
static const float cos_q[5] = {0.9950041653f,0.9800665778f,0.9553364891f,0.9950041653f,0.9887710779f};

#if XPAR_XF_SOLVER_LM_SO101_0_BASEADDR != 0x40000000U || XPAR_XF_SOLVER_LM_SO101_1_BASEADDR != 0x40010000U
#error Wrong hardware platform: expected RK7020 dual solver addresses
#endif
static const UINTPTR cores[2] = {XPAR_XF_SOLVER_LM_SO101_0_BASEADDR,
                                XPAR_XF_SOLVER_LM_SO101_1_BASEADDR};
static const float reference[5] = {0.0027827177f,0.0658248156f,-0.0880604611f,0.0217207295f,0.0000000000f};
static const float error_vector[6] = {.02f,.015f,.01f,0,0,0};
volatile int ik_test_stage = 0; /* 10: before first AXI read; 20: started; 100: all pass */
volatile int ik_test_core = -1;
volatile int ik_test_case = -1;
volatile int ik_test_failures = 0;
volatile u32 ik_last_ctrl = 0;
volatile u32 ik_last_status = 0;
volatile float ik_last_delta[5];

static float magnitude(float x) { return x < 0 ? -x : x; }
static void write_float(UINTPTR addr, float value)
{
    u32 bits; memcpy(&bits, &value, sizeof bits); Xil_Out32(addr,bits);
}
static int read_float(UINTPTR addr, float *value)
{
    u32 bits = Xil_In32(addr);
    memcpy(value,&bits,sizeof bits);
    return (bits & 0x7f800000U) != 0x7f800000U;
}
static void print_delta(int joint, float value)
{
    char line[100];
    /* Integer formatting works even with BSPs without printf float support.
       Caller checks finite and limits magnitude before conversion. */
    unsigned long scaled=(unsigned long)(magnitude(value)*1000000.0f+0.5f);
    snprintf(line,sizeof line,"  d%d = %s%lu.%06lu rad\r\n",joint,
             value < 0 ? "-" : "",scaled/1000000UL,scaled%1000000UL);
    send_text(line);
}
static int wait_ctrl(UINTPTR base,u32 mask)
{
    /* This bounds polling, but cannot recover a stalled AXI bus transaction.
       If paused inside Xil_In32, check bitstream, PS FCLK and reset first. */
    for (unsigned int n=0;n<100000U;++n) {
        ik_last_ctrl=Xil_In32(base+REG_CTRL);
        if (ik_last_ctrl & mask) return 1;
        usleep(10);
    }
    return 0;
}
static int run_case(int core,int test,float result[5])
{
    UINTPTR base=cores[core];
    char line[128];
    int ok=1;
    ik_test_core=core;ik_test_case=test;ik_test_stage=10;
    snprintf(line,sizeof line,"CORE%d CASE%d base=0x%08lx: %s\r\n",core,test,
             (unsigned long)base,test==1?"zero error":"C reference input");
    send_text(line);
    if (!wait_ctrl(base,4U)) {
        send_text("FAIL: IP not idle; check PL bitstream/clock/reset.\r\n");
        ik_test_stage=-10;return 0;
    }
    for(int i=0;i<5;++i) {
        write_float(base+sin_offset[i],sin_q[i]);
        write_float(base+cos_offset[i],cos_q[i]);
    }
    for(int i=0;i<6;++i)write_float(base+e_offset[i],test==1?0.0f:error_vector[i]);
    write_float(base+REG_LAMBDA,.01f);
    Xil_Out32(base+REG_STATE_I,0U);
    ik_test_stage=20;
    Xil_Out32(base+REG_CTRL,1U); /* ap_start; no auto restart */
    if (!wait_ctrl(base,2U)) { /* ap_done is clear-on-read */
        send_text("FAIL: completion timeout.\r\n");ik_test_stage=-20;return 0;
    }
    ik_last_status=Xil_In32(base+REG_STATUS);
    snprintf(line,sizeof line,"  status=%lu\r\n",(unsigned long)ik_last_status);
    send_text(line);
    if(ik_last_status!=0)ok=0;
    for(int i=0;i<5;++i) {
        int finite=read_float(base+d_offset[i],&result[i]);
        float expected=test==1?0.0f:reference[i];
        ik_last_delta[i]=result[i];
        if (!finite || magnitude(result[i])>1000.0f) {
            send_text("  FAIL: non-finite/out-of-range delta\r\n");ok=0;continue;
        }
        print_delta(i,result[i]);
        if (magnitude(result[i]-expected)>2e-6f+2e-3f*magnitude(expected))ok=0;
    }
    send_text(ok?"  CASE PASS\r\n":"  CASE FAIL: status or numerical mismatch\r\n");
    ik_test_stage=ok?30:-30;
    return ok;
}
static void test_ik(void)
{
    float saved[2][5],out[5];
    int cross_ok=1;
    send_text("\r\nRK7020 IK TEST 135MM v2: two cores, three cases each; no servo commands\r\n");
    send_text("Requires matching SO101_135MM bit + PS initialization; old 155mm bit is incompatible.\r\n");
    for(int core=0;core<2;++core) {
        for(int test=0;test<3;++test) {
            if(!run_case(core,test,out)) {
                ++ik_test_failures;
                send_text("OVERALL FAIL: stopped at first failing case.\r\n");return;
            }
            if(test==0)memcpy(saved[core],out,sizeof out);
            if(test==2)for(int i=0;i<5;++i)
                if(magnitude(out[i]-saved[core][i])>1e-6f)cross_ok=0;
        }
    }
    for(int i=0;i<5;++i)
        if(magnitude(saved[0][i]-saved[1][i])>1e-6f)cross_ok=0;
    if(!cross_ok) {
        ++ik_test_failures;ik_test_stage=-40;
        send_text("OVERALL FAIL: repeatability or cross-core mismatch.\r\n");
    } else {
        ik_test_stage=100;
        send_text("OVERALL PASS: 6 cases + repeatability + cross-core agreement.\r\n");
    }
}

#include "ik_iteration.h"
#include <math.h>
volatile int ik_iteration_stage=0;
volatile int ik_iteration_number=0;
volatile double ik_iteration_energy=0;
volatile float ik_iteration_q[5];
static ik_result iteration_results[2][2];

static int hardware_step(void *context,const float q[5],const float e[6],float damping,float delta[5])
{
    int core=*(int *)context;UINTPTR base=cores[core];
    ik_test_core=core;ik_test_stage=10;
    if(!wait_ctrl(base,4U)) {send_text("ITER FAIL: IP not idle\r\n");return 1;}
    for(int i=0;i<5;++i) {
        write_float(base+sin_offset[i],sinf(q[i]));
        write_float(base+cos_offset[i],cosf(q[i]));
    }
    for(int i=0;i<6;++i)write_float(base+e_offset[i],e[i]);
    write_float(base+REG_LAMBDA,damping);Xil_Out32(base+REG_STATE_I,0);
    ik_test_stage=20;Xil_Out32(base+REG_CTRL,1U);
    if(!wait_ctrl(base,2U)) {send_text("ITER FAIL: ap_done timeout\r\n");return 1;}
    ik_last_status=Xil_In32(base+REG_STATUS);
    if(ik_last_status) {send_text("ITER FAIL: IP status nonzero\r\n");return 1;}
    for(int i=0;i<5;++i)if(!read_float(base+d_offset[i],&delta[i]))return 1;
    return 0;
}
static void iteration_trace(void *context,int n,const float q[5],double E,double p,double r,float damping)
{
    char line[220];int core=*(int *)context;
    ik_iteration_number=n;ik_iteration_energy=E;
    for(int i=0;i<5;++i)ik_iteration_q[i]=q[i];
    if(n<3 || n%10==0 || E<1e-10) {
        snprintf(line,sizeof line,"  core=%d iter=%d E=%.6e pos=%.6e m rot=%.6e rad lambda=%.6e\r\n",
                 core,n,E,p,r,(double)damping);send_text(line);
        snprintf(line,sizeof line,"    q=[%.6f %.6f %.6f %.6f %.6f] rad\r\n",
                 (double)q[0],(double)q[1],(double)q[2],(double)q[3],(double)q[4]);send_text(line);
    }
}
static void test_iterations(void)
{
    /* Change target_q and initial_q here for additional reachable-pose tests.
       target = FK(target_q); target_q is NOT used to compute an update. */
    const float target_q[2][5]={{.2f,-.3f,.4f,-.2f,.15f},{-.25f,.2f,-.35f,.15f,-.2f}};
    const float initial_q[2][5]={{.1f,-.2f,.3f,-.1f,0},{0,0,0,0,0}};
    const char *reasons[]={"CONVERGED","MAX_ITER","HARDWARE_ERROR","NUMERICAL_ERROR"};
    char line[240];int passed=1;
    ik_iteration_stage=1;
    send_text("\r\nFULL ITERATION v1: SO101 135mm model; mask=all ones; no servo commands\r\n");
    send_text("E tolerance=1e-10; max updates=1000; k=0.01; no line search\r\n");
    for(int scenario=0;scenario<2;++scenario) {
        double target[16];ik_fk(target_q[scenario],target);
        snprintf(line,sizeof line,"SCENARIO%d method=%s goal_xyz=[%.6f %.6f %.6f] m\r\n",scenario,
                 scenario?"sugihara":"wampler",target[3],target[7],target[11]);send_text(line);
        for(int core=0;core<2;++core) {
            ik_result *res=&iteration_results[scenario][core];
            int code=ik_solve(target,initial_q[scenario],1000,1e-10,.01f,scenario,
                             hardware_step,iteration_trace,&core,res);
            snprintf(line,sizeof line,"SCENARIO%d CORE%d %s updates=%d E=%.6e pos=%.6e m rot=%.6e rad\r\n",
                     scenario,core,reasons[code],res->iterations,res->energy,res->position_error,res->rotation_error);
            send_text(line);
            if(code) {passed=0;if(code>=2) {ik_iteration_stage=-code;return;}}
        }
        for(int j=0;j<5;++j)if(fabsf(iteration_results[scenario][0].q[j]-iteration_results[scenario][1].q[j])>1e-4f)passed=0;
    }
    ik_iteration_stage=passed?100:-1;
    send_text(passed?"FULL ITERATION PASS: both targets converged on both cores.\r\n":
                     "FULL ITERATION FAIL: max iterations or cross-core mismatch.\r\n");
}

#include "pose_uart.inc"

int main(void)
{
    XUartPs_Config *cfg;
    XUartPsFormat fmt = {115200, XUARTPS_FORMAT_8_BITS,
                        XUARTPS_FORMAT_NO_PARITY, XUARTPS_FORMAT_1_STOP_BIT};
    uart_test_stage = 1;
#ifdef SDT
    cfg = XUartPs_LookupConfig(RK_UART0_BASE);
#else
    cfg = XUartPs_LookupConfig(XPAR_XUARTPS_0_DEVICE_ID);
#endif
    if (!cfg) fail(-1);
    if (cfg->BaseAddress != RK_UART0_BASE) fail(-2);
    uart_test_stage = 2;
    if (XUartPs_CfgInitialize(&uart, cfg, cfg->BaseAddress) != XST_SUCCESS)
        fail(-3);
    if (XUartPs_SetDataFormat(&uart, &fmt) != XST_SUCCESS) fail(-4);
    XUartPs_SetOperMode(&uart, XUARTPS_OPER_MODE_NORMAL);
    XUartPs_SetOptions(&uart, 0U);

    uart_test_stage = 100;
    servo_init();
    motor_setup();
    arm_setup();
    cal_setup();
    send_text("UART0 ready. Starting IK test once.\r\n");
    test_ik();
    if (ik_test_stage == 100 && ik_test_failures == 0) test_iterations();
    else send_text("Skipping iterations: single-step tests failed.\r\n");
    if (ik_iteration_stage == 100) pose_server();
    else send_text("POSE1 DISABLED: self-test failed\r\n");
    for (;;) {
        ++uart_test_heartbeat;
        usleep(1000000);
    }
}
