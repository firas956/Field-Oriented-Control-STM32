#include "app/motor_control.h"
#include "hw/hw_adc.h"
#include "hw/hw_hall_sensor.h"
#include "hw/hw_pwm.h"

FOC_Controller_t foc_core;
static PI_Controller_t id_controller;
static PI_Controller_t iq_controller;

void MotorControl_Init(void) {
    // 1. Zero out our telemetry structure state
    foc_core.id_target = 0.0f; 
    foc_core.iq_target = 0.0f; 
    foc_core.vdc_bus   = 24.0f; 

    // 2. Initialize PI Loops for 10 kHz execution (Ts = 0.0001s)
    // Adjust Kp and Ki based on your specific motor phase resistance/inductance
    float Ts = 1.0f / 10000.0f; 
    
    // Output limits match max modulation index constraints (e.g., -15V to +15V duty boundaries)
    PI_Init(&id_controller, 1.5f, 200.0f, Ts, -24.0f, 24.0f);
    PI_Init(&iq_controller, 1.5f, 200.0f, Ts, -24.0f, 24.0f);


}

void MotorControl_RunIteration(void) {

    HW_ADC_ReadCurrents(&foc_core.i_abc.a, &foc_core.i_abc.b);
    foc_core.i_abc.c = -(foc_core.i_abc.a + foc_core.i_abc.b);
    foc_core.angle_rad = HW_Hall_GetElectricalAngle();
    FOC_Clark(&foc_core.i_abc, &foc_core.i_alphabeta);
    FOC_Park(&foc_core.i_alphabeta, &foc_core.i_dq, foc_core.angle_rad);
    float id_error = foc_core.id_target - foc_core.i_dq.d;
    float iq_error = foc_core.iq_target - foc_core.i_dq.q;
    foc_core.v_dq.d = PI_Update(&id_controller, id_error);
    foc_core.v_dq.q = PI_Update(&iq_controller, iq_error);
    FOC_InversePark(&foc_core.v_dq, &foc_core.v_alphabeta, foc_core.angle_rad);
    FOC_SVPWM(&foc_core.v_alphabeta, foc_core.vdc_bus, &foc_core.duty_cycles);
    HW_PWM_SetDuties(&foc_core.duty_cycles);
}

void MotorControl_SetTorqueTarget(float iq_amps) {
    foc_core.iq_target = iq_amps;
}