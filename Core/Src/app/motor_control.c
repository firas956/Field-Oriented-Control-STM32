#include "app/motor_control.h"
#include "app/motor_config.h"
#include "core/lpf.h"
#include "hw/hw_adc.h"
#include "hw/hw_hall_sensor.h"
#include "hw/hw_pwm.h"
#include "core/hall_pll.h"
#include "core/modulator.h"
#include "core/smc.h"
#include "core/smc_current.h"
#include "core/speed_ref.h"
#include "core/offset_compensator.h"
#include "core/harm2_compensator.h"
#include "core/harm6_compensator.h"
#include <math.h>

#define MOTOR_POLE_PAIRS 2
#define TWO_PI           6.28318530718f

#define IQ_LIMIT_A      6.0f

static const PWM_Modulator_t PWM_Modulate = FOC_SVPWM;

FOC_Controller_t foc_core;
static PI_Controller_t id_controller;
static PI_Controller_t iq_controller;
static PI_Controller_t speed_controller;
static SMC_Controller_t speed_smc;
static CSMC_Controller_t current_smc;
static SpeedRef_t speed_ref_gen;
static LPC_Filter_t filtered_ia;
static LPC_Filter_t filtered_ib;

/* ---- Speed-controller strategy selection (mirrors PWM_Modulate) ----
 * Signature: mechanical reference & measurement [RPM] -> iq* [A].
 * Switch PID <-> sliding mode with the one SpeedControl line below.       */
typedef float (*SpeedController_t)(float ref_rpm, float meas_rpm);
static float Speed_PI (float ref_rpm, float meas_rpm);
static float Speed_SMC(float ref_rpm, float meas_rpm);
static const SpeedController_t SpeedControl = Speed_SMC;   /* Speed_PI or Speed_SMC */

/* ---- Current-controller strategy selection (mirrors PWM_Modulate) ----
 * Signature: dq references & measurements + electrical speed -> v_dq.
 * Switch PI <-> sliding mode with the one CurrentControl line below.      */
typedef void (*CurrentController_t)(const DQ_t *i_ref, const DQ_t *i_meas,
                                    float omega_elec, DQ_t *v_dq);
static void Current_PI (const DQ_t *i_ref, const DQ_t *i_meas, float we, DQ_t *v_dq);
static void Current_SMC(const DQ_t *i_ref, const DQ_t *i_meas, float we, DQ_t *v_dq);
static const CurrentController_t CurrentControl = Current_PI; /* Current_PI or Current_SMC */
// Tracking PLL turning the 60-degree hall steps into a continuous angle
static HallPLL_t hall_pll;

// Decimation counter to downsample the speed loop from 20kHz to 1kHz
static uint16_t speed_loop_counter = 0;

/* ---- One-shot datalogger: single channel @ 20 kHz, extracted via gdb ---- */
#define DATALOG_N 28672u                  /* 112 KB float32: 1.43 s, df = 0.70 Hz */

volatile uint32_t datalog_armed = 0;      /* write 1 from debugger to start */
volatile uint32_t datalog_done  = 0;      /* set by ISR when buffer is full */
static uint32_t   datalog_index = 0;
//static float ia_f = 0.0f;
//static float ib_f = 0.0f;

float datalog_buf[DATALOG_N];

/* Phase-current offset trim; see core/offset_compensator.h for the method.
 * Non-static so a gdb session / the datalogger can watch curr_offset.off_a. */
#define IOFF_MIN_WE_RAD_S   100.0f  /* PLL trustworthy from ~480 rpm mech    */
#define IOFF_MAX_SAMPLES    4000u   /* window guard: 200 ms at 20 kHz        */
#define IOFF_SPEED_TOL_RPM  50.0f   /* |target - measured| for "settled"     */
#define IOFF_ALPHA          0.15f   /* per-period correction fraction        */
#define IOFF_CLAMP_A        0.5f    /* sanity clamp; real drift is << this   */

OffsetComp_t curr_offset;

/* 2nd-harmonic (2*we) trim in dq; see core/harm2_compensator.h for the method.
 * Non-static so a gdb session / the datalogger can watch harm2_comp.aq. */
#define IH2_MIN_WE_RAD_S    100.0f  /* same PLL-trust floor as the offset trim */
#define IH2_MIN_MAG_A       0.20f   /* below this |idq| the window is noise     */
#define IH2_MAX_SAMPLES     4000u   /* window guard: 200 ms at 20 kHz           */
#define IH2_ALPHA           0.10f   /* per-period correction fraction           */
#define IH2_CLAMP           0.15f   /* 15% of |idq|; a real mismatch is << this */
#define IH2_MAG_BETA        0.001f  /* |idq| tracker: 20 rad/s, 1/10 of 2*we_min */

Harm2Comp_t harm2_comp;

/* 6th-harmonic (6*we) trim in dq; see core/harm6_compensator.h for the method.
 * Non-static so a gdb session / the datalogger can watch harm6_comp.aq. */
#define IH6_MIN_WE_RAD_S    100.0f  /* same PLL-trust floor as the other trims */
#define IH6_MAX_SAMPLES     4000u   /* window guard: 200 ms at 20 kHz          */
#define IH6_ALPHA           0.10f   /* per-period correction fraction          */
#define IH6_CLAMP_A         0.50f   /* ~3x the 0.15 A a 0.3 V dead-time term
                                     * drives through Z(6we) = 2.3..2.9 ohm    */

Harm6Comp_t harm6_comp;

/* Breakpoint anchor: gdb breakpoint goes on the line inside this function */
__attribute__((noinline)) static void Datalog_CaptureDone(void)
{
    datalog_done = 1;
}

/* Arm a fresh capture from the main loop (no-op while one is in progress) */
void Datalog_Arm(void)
{
    if (!datalog_armed) {
        datalog_index = 0;
        datalog_done  = 0;
        datalog_armed = 1;   /* written last: this is what gates the ISR */
        
        
        
    }
}

void MotorControl_Init(void) {
    foc_core.id_target = 0.0f;
    foc_core.iq_target = 0.0f;
    foc_core.vdc_bus   = 24.0f;
    foc_core.speed_command    = 0.0f;
    foc_core.speed_target     = 0.0f;
    foc_core.speed_target_dot = 0.0f;
    foc_core.speed_measured = 0.0f;

    // Current Loop Time Step (20 kHz)
    float Ts = 1.0f / 20000.0f;

    // Maximum phase voltage available in the linear SVPWM range is
    // Vdc/sqrt(3) (~13.9 V at 24 V bus), NOT Vdc. d and q share this circle.
    float v_max = foc_core.vdc_bus * 0.57735027f;

    // Initialize Current PI Loops (gains to be tuned from measured R and L:
    
    float w_bw = TWO_PI * I_LOOP_BW_HZ;
   /* PI_Init(&id_controller, 1.5f, 200.0f, Ts, -v_max, v_max);
    PI_Init(&iq_controller, 1.5f, 200.0f, Ts, -v_max, v_max);*/
    PI_Init(&id_controller, MOTOR_L * w_bw, MOTOR_R_PH * w_bw, Ts, -v_max, v_max);
    PI_Init(&iq_controller, MOTOR_L * w_bw, MOTOR_R_PH * w_bw, Ts, -v_max, v_max);

    // Speed Loop (1 kHz). Units: error in RPM, output in amps.
    // 0.005 A/RPM: a 200 RPM error requests 1 A. Starting point - tune on the bench.
    float Ts_speed = 1.0f / 1000.0f;
    PI_Init(&speed_controller, 0.002f, 0.022f, Ts_speed, -IQ_LIMIT_A, IQ_LIMIT_A);

    // Discrete Integral Sliding-Mode speed controller (alternative to the PI).
    // Plant: J, F from mechanical ID; Kt = 1.5*p*psi_r = 1.5*2*0.02 = 0.06.
    // Gains start ~ equivalent to the working PI, plus the switching robustness.
    SMC_Init(&speed_smc,
             3.0e-5f,    // J   [kg.m^2]
             7.0e-5f,    // F   [N.m.s/rad]
             0.06f,      // Kt  [N.m/A] = 1.5*p*lambda
             Ts_speed,   // Ts  [s]
             1.0f,       // c1
             0.02f,      // c2   (integral weight)
             0.98f,      // eta = 1 - q*Ts   (q = 20 1/s)
             200.0f,     // k    (k*Ts = 0.2 < 1)
             2.0f,       // phi  boundary layer [rad/s] (~19 rpm)
             -IQ_LIMIT_A, IQ_LIMIT_A);

    // Discrete Integral Sliding-Mode current controller (alternative to the
    // id/iq PIs). Gains chosen so the LINEAR part reproduces the PI above:
    //   Kp = q*L + R, Ki = (c2/c1)*q*L  ->  same wn = 622 rad/s, zeta = 1.
    // On top it adds the exact R*i + back-EMF feed-forward and a
    // +/-(k*L/c1) V switching term (0.80 V) to absorb dead-time / model error.
    CSMC_Init(&current_smc,
              MOTOR_L, MOTOR_R_PH, lambda, Ts,
              1.0f,                          // c1
              MOTOR_R_PH * Ts / MOTOR_L,     // c2 = c1*R*Ts/L (cancels the L/R pole)
              1.0f - w_bw * Ts,              // eta = 1 - q*Ts, q = 2*pi*I_LOOP_BW_HZ
              150.0f,                        // k   -> +/-0.80 V switching authority
              0.1f,                          // phi boundary layer [A]
              v_max);

    // Smooth speed reference: triple integrator -> W/Wtgt = p^3/(s+p)^3.
    // p = 30 rad/s gives a monotonic, zero-overshoot transition in ~200 ms,
    // well inside the closed-loop wn = 51 rad/s so the motor can track it.
    // The limits are safety clamps only (a normal 700 rpm step peaks at
    // ~5700 rpm/s, ~0.6e6 rpm/s^2), never active in normal operation.
    SpeedRef_Init(&speed_ref_gen, Ts_speed,
                  30.0f,      // pole p [rad/s]  -> transition ~6/p = 200 ms
                  20000.0f,   // a_max [rpm/s]
                  2.0e6f,     // j_max [rpm/s^2]
                  2.0e8f);    // s_max [rpm/s^3]
    //LPF_Init(&filtered_ia, 0.1, 0);
    //LPF_Init(&filtered_ib, 0.1, 0);
    //LPF_Init(&speed_controller, 0.1, 0);
    // PLL: wn = 125.7 rad/s (20 Hz), zeta = 1.0 -> ki = wn^2, kp = 2*wn.
    // Valid down to ~500 RPM (needs ~5 hall edges per PLL time constant).
    HallPLL_Init(&hall_pll, 251.3f, 15791.0f, Ts);
    speed_loop_counter = 0;
    OffsetComp_Init(&curr_offset, IOFF_MIN_WE_RAD_S, IOFF_ALPHA,
                    IOFF_CLAMP_A, IOFF_MAX_SAMPLES);
    Harm2Comp_Init(&harm2_comp, IH2_MIN_WE_RAD_S, IH2_MIN_MAG_A, IH2_ALPHA,
                   IH2_CLAMP, IH2_MAG_BETA, IH2_MAX_SAMPLES);
    Harm6Comp_Init(&harm6_comp, IH6_MIN_WE_RAD_S, IH6_ALPHA,
                   IH6_CLAMP_A, IH6_MAX_SAMPLES);
}

/*
 * Re-arm the controller from the current rotor position. Called by the state
 * machine when entering STATE_RUNNING so that stale integrators / a stale
 * PLL angle from IDLE time can never drive the first PWM cycles.
 */
void MotorControl_Reset(void) {
    PI_Reset(&id_controller, 0.0f);
    PI_Reset(&iq_controller, 0.0f);
    PI_Reset(&speed_controller, 0.0f);
    SMC_Reset(&speed_smc);
    CSMC_Reset(&current_smc);
    SpeedRef_Reset(&speed_ref_gen, foc_core.speed_measured);  /* bumpless start */
    HallPLL_Reset(&hall_pll, HW_Hall_GetBaseAngle());
    /* Drop the averaging window but KEEP the estimate: the sensor drift is
     * thermal, so the last estimate is still the best one after a restart. */
    OffsetComp_Reset(&curr_offset, HW_Hall_GetBaseAngle());
    /* Same policy: the normalized shape is a hardware property, so it
     * survives the restart; only the window and the |idq| scale are dropped. */
    Harm2Comp_Reset(&harm2_comp, HW_Hall_GetBaseAngle());
    Harm6Comp_Reset(&harm6_comp, HW_Hall_GetBaseAngle());
    foc_core.iq_target = 0.0f;
    speed_loop_counter = 0;
}

void MotorControl_RunIteration(void) {

    // Update the tracking PLL from the coarse 60-degree hall angle
    float coarse_hall_angle = HW_Hall_GetBaseAngle();
    HallPLL_Update(&hall_pll, coarse_hall_angle);

    foc_core.angle_rad = hall_pll.est_angle;

    // Convert electrical rad/s from PLL into mechanical RPM
    float omega_elec = hall_pll.est_speed_lpf;
    foc_core.speed_measured = (omega_elec * 60.0f) / (TWO_PI * (float)MOTOR_POLE_PAIRS);

    // Phase current acquisition (synchronized to the PWM zero vector)
    HW_ADC_ReadCurrents(&foc_core.i_abc.a, &foc_core.i_abc.b);
    
    OffsetComp_Apply(&curr_offset, &foc_core.i_abc.a, &foc_core.i_abc.b);
    int ioff_settled = (fabsf(foc_core.speed_target - foc_core.speed_measured)<= IOFF_SPEED_TOL_RPM);
    OffsetComp_Update(&curr_offset, foc_core.i_abc.a, foc_core.i_abc.b,foc_core.angle_rad, omega_elec, ioff_settled);
    
    foc_core.i_abc.c = -(foc_core.i_abc.a + foc_core.i_abc.b);

    FOC_Clark(&foc_core.i_abc, &foc_core.i_alphabeta);
    FOC_Park(&foc_core.i_alphabeta, &foc_core.i_dq, foc_core.angle_rad);

    /* Harmonic trims, on the same 'settled' gate as the abc offset trim. All
     * three are orthogonal over the averaging window, so they converge
     * independently -- uncomment the 2*we pair to run both at once. */
    
    Harm2Comp_Apply(&harm2_comp, &foc_core.i_dq, foc_core.angle_rad);
    Harm2Comp_Update(&harm2_comp, &foc_core.i_dq, foc_core.angle_rad,
                     omega_elec, ioff_settled);
    
    /*
    Harm6Comp_Apply(&harm6_comp, &foc_core.i_dq, foc_core.angle_rad);
    Harm6Comp_Update(&harm6_comp, &foc_core.i_dq, foc_core.angle_rad,
                     omega_elec, ioff_settled);
       */              
    speed_loop_counter++;
    if (speed_loop_counter >= 20) {
        speed_loop_counter = 0;
        
        foc_core.speed_target     = SpeedRef_Update(&speed_ref_gen, foc_core.speed_command);
        foc_core.speed_target_dot = speed_ref_gen.a;

        foc_core.iq_target = SpeedControl(foc_core.speed_target, foc_core.speed_measured);

    }
    
    //foc_core.iq_target = 0.4;
    // Current PI Controllers
    DQ_t i_ref = { foc_core.id_target, foc_core.iq_target };
    CurrentControl(&i_ref, &foc_core.i_dq, omega_elec, &foc_core.v_dq);
    
    float v_max = foc_core.vdc_bus * 0.57735027f;
    float vq_headroom_sq = v_max * v_max - foc_core.v_dq.d * foc_core.v_dq.d;
    float vq_max = (vq_headroom_sq > 0.0f) ? sqrtf(vq_headroom_sq) : 0.0f;
    if (foc_core.v_dq.q >  vq_max) foc_core.v_dq.q =  vq_max;
    if (foc_core.v_dq.q < -vq_max) foc_core.v_dq.q = -vq_max;
    
    FOC_InversePark(&foc_core.v_dq, &foc_core.v_alphabeta, foc_core.angle_rad);

    PWM_Modulate(&foc_core.v_alphabeta, foc_core.vdc_bus, &foc_core.duty_cycles);
    HW_PWM_SetDuties(&foc_core.duty_cycles);

    /* ---- Datalog: keep exactly ONE channel line uncommented ---- */
    if (datalog_armed) {
        if (datalog_index < DATALOG_N) {
          
            datalog_buf[datalog_index++] = foc_core.speed_measured;
            //datalog_buf[datalog_index++] = foc_core.i_dq.d;
            //datalog_buf[datalog_index++] = foc_core.i_dq.q;
            //datalog_buf[datalog_index++] = foc_core.v_dq.q ;
            //datalog_buf[datalog_index++] = foc_core.angle_rad;
            //datalog_buf[datalog_index++] = foc_core.iq_target;
            //datalog_buf[datalog_index++] = ia_f;
            //datalog_buf[datalog_index++] = foc_core.i_abc.a;
            
            
        } else {
            datalog_armed = 0;
            Datalog_CaptureDone();
        }
    }
}

void MotorControl_SetTorqueTarget(float iq_amps) {
    foc_core.iq_target = iq_amps;
}

void MotorControl_SetSpeedTarget(float speed_rpm) {
    foc_core.speed_command = speed_rpm;   /* shaped into speed_target at 1 kHz */
}

/* ---- Speed-controller strategy wrappers (see SpeedControl near the top) ---- */
static float Speed_PI(float ref_rpm, float meas_rpm) {
    return PI_Update(&speed_controller, ref_rpm - meas_rpm);
}
static float Speed_SMC(float ref_rpm, float meas_rpm) {
    const float RPM2RAD = TWO_PI / 60.0f;            // mechanical rpm -> rad/s
    return SMC_Update(&speed_smc, ref_rpm * RPM2RAD, meas_rpm * RPM2RAD);
}

/* ---- Current-controller strategy wrappers (see CurrentControl near the top) ---- */
static void Current_PI(const DQ_t *i_ref, const DQ_t *i_meas, float we, DQ_t *v_dq) {
    (void)we;                                        // PI has no decoupling term
    v_dq->d = PI_Update(&id_controller, i_ref->d - i_meas->d);
    v_dq->q = PI_Update(&iq_controller, i_ref->q - i_meas->q);
}
static void Current_SMC(const DQ_t *i_ref, const DQ_t *i_meas, float we, DQ_t *v_dq) {
    CSMC_Update(&current_smc, i_ref, i_meas, we, v_dq);
}
