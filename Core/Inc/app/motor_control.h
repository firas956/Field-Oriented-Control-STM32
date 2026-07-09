#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "core/foc_math.h"
#include "core/pid.h"

// The main FOC execution call step executed inside your high-frequency ISR
typedef struct {
    float id_target;
    float iq_target;
    float vdc_bus;

    Phase_t i_abc;
    float angle_rad;
    AlphaBeta_t i_alphabeta;
    DQ_t        i_dq;
    DQ_t        v_dq;
    AlphaBeta_t v_alphabeta;
    Phase_t     duty_cycles;
}FOC_Controller_t;

void MotorControl_Init(void);
void MotorControl_RunIteration(void);
void MotorControl_SetTorqueTarget(float iq_amps);

extern FOC_Controller_t foc_core;

#endif