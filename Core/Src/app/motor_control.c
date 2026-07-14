#include "app/motor_control.h"
#include "app/motor_config.h"
#include "app/state_machine.h"
#include "hw/hw_adc.h"
#include "hw/hw_hall_sensor.h"
#include "hw/hw_pwm.h"
#include "core/lpf.h"
#include <math.h>

FOC_Controller_t foc_core;
static PI_Controller_t id_controller;
static PI_Controller_t iq_controller;
static PI_Controller_t speed_controller;
static LPC_Filter_t speed_filter;
static uint32_t speed_loop_divider = 0;

void MotorControl_Init(void) {
    // 1. Zero out our telemetry structure state
    foc_core.id_target = 0.0f;
    foc_core.iq_target = 0.0f;
    foc_core.vdc_bus   = VBUS_VOLTS;
    foc_core.speed_target = 0.0f;   // Initialize speed target
    foc_core.speed_measured = 0.0f; // Initialize measured speed

    // 2. Initialize PI Loops for 10 kHz execution (Ts = 0.0001s)
    // Adjust Kp and Ki based on your specific motor phase resistance/inductance
    float Ts = 1.0f / CURRENT_LOOP_FREQ_HZ;

    // Current-loop output limits: the linear SVPWM range is Vdc/sqrt(3) per
    // voltage vector. +/-24 V per axis was physically unreachable and only
    // served to wind the integrators deep into saturation.
    float v_lim = VDQ_LIMIT_FRACTION * VBUS_VOLTS * ONE_BY_SQRT3;
    PI_Init(&id_controller, 1.5f, 200.0f, Ts, -v_lim, v_lim);
    PI_Init(&iq_controller, 1.5f, 200.0f, Ts, -v_lim, v_lim);

    // Speed loop runs decimated inside RunIteration, so Ts must match the
    // decimated rate, and the iq clamp must stay below the supply limit.
    float Ts_speed = (float)SPEED_LOOP_DECIMATION / CURRENT_LOOP_FREQ_HZ;
    PI_Init(&speed_controller, 0.5f, 0.01f, Ts_speed, -IQ_LIMIT_AMPS, IQ_LIMIT_AMPS);
    LPF_Init(&speed_filter, SPEED_LPF_ALPHA, 0.0f);
}

void MotorControl_RunIteration(void) {

    HW_ADC_ReadCurrents(&foc_core.i_abc.a, &foc_core.i_abc.b);
    foc_core.i_abc.c = -(foc_core.i_abc.a + foc_core.i_abc.b);

    // Software overcurrent trip: kill the gates within this same 100 us cycle
    // and latch the fault. Only an explicit request back to IDLE re-arms.
    if (fabsf(foc_core.i_abc.a) > OC_TRIP_AMPS ||
        fabsf(foc_core.i_abc.b) > OC_TRIP_AMPS ||
        fabsf(foc_core.i_abc.c) > OC_TRIP_AMPS) {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_TIMER_HANDLE);
        StateMachine_RequestState(STATE_FAULT);
    }

    if (StateMachine_GetState() != STATE_RUNNING) {
        // Gates are off (IDLE/FAULT): hold the controllers in reset so they do
        // not wind up against a bridge that is not conducting, and park the
        // duties at the neutral zero vector for a clean re-arm.
        PI_Reset(&id_controller, 0.0f);
        PI_Reset(&iq_controller, 0.0f);
        PI_Reset(&speed_controller, 0.0f);
        speed_loop_divider = 0;
        foc_core.duty_cycles.a = 0.5f;
        foc_core.duty_cycles.b = 0.5f;
        foc_core.duty_cycles.c = 0.5f;
        HW_PWM_SetDuties(&foc_core.duty_cycles);
        return;
    }

    // Speed loop decimated to the rate its gains were designed for (1 kHz)
    if (++speed_loop_divider >= SPEED_LOOP_DECIMATION) {
        speed_loop_divider = 0;
        float raw_rpm = HW_Hall_GetSpeedRPM(MOTOR_POLE_PAIRS);
        foc_core.speed_measured = LPF_Update(&speed_filter, raw_rpm);
        float speed_error = foc_core.speed_target - foc_core.speed_measured;
        foc_core.iq_target = PI_Update(&speed_controller, speed_error);
    }

    foc_core.angle_rad = HW_Hall_GetElectricalAngle();

    FOC_Clark(&foc_core.i_abc, &foc_core.i_alphabeta);
    FOC_Park(&foc_core.i_alphabeta, &foc_core.i_dq, foc_core.angle_rad);

    float id_error = foc_core.id_target - foc_core.i_dq.d;
    float iq_error = foc_core.iq_target - foc_core.i_dq.q;
    foc_core.v_dq.d = PI_Update(&id_controller, id_error);
    foc_core.v_dq.q = PI_Update(&iq_controller, iq_error);

    // Circular voltage limit with d-axis priority: |Vdq| must stay inside the
    // linear SVPWM range, and the flux axis keeps its authority when saturated.
    float v_max = VDQ_LIMIT_FRACTION * foc_core.vdc_bus * ONE_BY_SQRT3;
    if (foc_core.v_dq.d > v_max) {
        foc_core.v_dq.d = v_max;
    } else if (foc_core.v_dq.d < -v_max) {
        foc_core.v_dq.d = -v_max;
    }
    float vq_max = sqrtf(v_max * v_max - foc_core.v_dq.d * foc_core.v_dq.d);
    if (foc_core.v_dq.q > vq_max) {
        foc_core.v_dq.q = vq_max;
    } else if (foc_core.v_dq.q < -vq_max) {
        foc_core.v_dq.q = -vq_max;
    }

    FOC_InversePark(&foc_core.v_dq, &foc_core.v_alphabeta, foc_core.angle_rad);

    FOC_SVPWM(&foc_core.v_alphabeta, foc_core.vdc_bus, &foc_core.duty_cycles);
    HW_PWM_SetDuties(&foc_core.duty_cycles);
}

void MotorControl_SetTorqueTarget(float iq_amps) {
    foc_core.iq_target = iq_amps;
}
void MotorControl_SetSpeedTarget(float speed_rpm) {
    foc_core.speed_target = speed_rpm;
}
