#include "app/motor_control.h"
#include "app/motor_config.h"
#include "hw/hw_adc.h"
#include "hw/hw_hall_sensor.h"
#include "hw/hw_pwm.h"
#include "core/hall_pll.h"
#include "core/modulator.h"
#include <math.h>

#define MOTOR_POLE_PAIRS 2
#define TWO_PI           6.28318530718f

#define IQ_LIMIT_A       6.0f

static const PWM_Modulator_t PWM_Modulate = FOC_SVPWM;

FOC_Controller_t foc_core;
static PI_Controller_t id_controller;
static PI_Controller_t iq_controller;
static PI_Controller_t speed_controller;

// Tracking PLL turning the 60-degree hall steps into a continuous angle
static HallPLL_t hall_pll;

// Decimation counter to downsample the speed loop from 20kHz to 1kHz
static uint16_t speed_loop_counter = 0;

/* ---- One-shot datalogger: single channel @ 20 kHz, extracted via gdb ---- */
#define DATALOG_N 28672u                  /* 112 KB float32: 1.43 s, df = 0.70 Hz */

volatile uint32_t datalog_armed = 0;      /* write 1 from debugger to start */
volatile uint32_t datalog_done  = 0;      /* set by ISR when buffer is full */
static uint32_t   datalog_index = 0;
float datalog_buf[DATALOG_N];

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
    foc_core.speed_target = 0.0f;
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
    PI_Init(&speed_controller, 0.005f, 0.02f, Ts_speed, -IQ_LIMIT_A, IQ_LIMIT_A);

    // PLL bandwidth ~10 Hz (wn = sqrt(ki) = 63 rad/s, zeta = kp/(2*sqrt(ki)) = 0.79)
    HallPLL_Init(&hall_pll, 377.0f, 35530.0f, Ts);
    speed_loop_counter = 0;
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
    HallPLL_Reset(&hall_pll, HW_Hall_GetBaseAngle());
    foc_core.iq_target = 0.0f;
    speed_loop_counter = 0;
}

void MotorControl_RunIteration(void) {

    // Update the tracking PLL from the coarse 60-degree hall angle
    float coarse_hall_angle = HW_Hall_GetBaseAngle();
    HallPLL_Update(&hall_pll, coarse_hall_angle);

    foc_core.angle_rad = hall_pll.est_angle;

    // Convert electrical rad/s from PLL into mechanical RPM
    float omega_elec = hall_pll.est_speed;
    foc_core.speed_measured = (omega_elec * 60.0f) / (TWO_PI * (float)MOTOR_POLE_PAIRS);

    // Phase current acquisition (synchronized to the PWM zero vector)
    HW_ADC_ReadCurrents(&foc_core.i_abc.a, &foc_core.i_abc.b);
    foc_core.i_abc.c = -(foc_core.i_abc.a + foc_core.i_abc.b);

    FOC_Clark(&foc_core.i_abc, &foc_core.i_alphabeta);
    FOC_Park(&foc_core.i_alphabeta, &foc_core.i_dq, foc_core.angle_rad);
    // Speed Control Loop at 1 kHz (every 20th iteration of the 20kHz loop)
    speed_loop_counter++;
    if (speed_loop_counter >= 20) {
        speed_loop_counter = 0;

        float speed_error = foc_core.speed_target - foc_core.speed_measured;
        foc_core.iq_target = PI_Update(&speed_controller, speed_error);
    }
    //foc_core.iq_target = 0.4;
    // Current PI Controllers
    float id_error = foc_core.id_target - foc_core.i_dq.d;
    float iq_error = foc_core.iq_target - foc_core.i_dq.q;
    foc_core.v_dq.d = PI_Update(&id_controller, id_error);
    foc_core.v_dq.q = PI_Update(&iq_controller, iq_error);

    /*float ed = -foc_core.i_dq.q * MOTOR_L *foc_core.speed_measured;
    float eq = ((foc_core.i_dq.d * MOTOR_L)+lambda) *foc_core.speed_measured;

    foc_core.v_dq.d = foc_core.v_dq.d + ed;
    foc_core.v_dq.q = foc_core.v_dq.q + eq;*/



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
            /*datalog_buf[datalog_index++] = foc_core.i_dq.d;*/
           /*datalog_buf[datalog_index++] = foc_core.i_dq.q;*/
            /*datalog_buf[datalog_index++] = foc_core.v_dq.q ;*/
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
    foc_core.speed_target = speed_rpm;
}
