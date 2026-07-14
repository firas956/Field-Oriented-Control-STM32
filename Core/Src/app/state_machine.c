#include "app/state_machine.h"
#include "app/motor_control.h"
#include "hw/hw_adc.h"
#include "hw/hw_pwm.h"

// volatile: written from the overcurrent trip in ISR context, read in the main loop
static volatile MotorState_t current_state = STATE_IDLE;

void StateMachine_Init(void) {
    current_state = STATE_IDLE;
    
    // Absolute safety default: Enforce immediate hardware gate shutdown
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_TIMER_HANDLE);
}

void StateMachine_Update(void) {
    
    switch (current_state) {
        case STATE_IDLE:
            // Keep inverter gates dead and target torque zeroed
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_TIMER_HANDLE);
            MotorControl_SetTorqueTarget(0.0f);
            break;

        case STATE_CALIBRATION:
            // Ensure gates are dead before calibrating
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_TIMER_HANDLE);
            MotorControl_SetTorqueTarget(0.0f);
            
            // Execute the blocking ADC offset calibration
            HW_ADC_Init();
            
            // Automatically return to IDLE so the system waits for a deliberate RUN command
            current_state = STATE_IDLE;
            break;

        case STATE_RUNNING:
            // Unmask the main output enable register to allow PWM to reach the IGBT gates
            __HAL_TIM_MOE_ENABLE(&PWM_TIMER_HANDLE);

            // Note: Torque is updated externally by calling MotorControl_SetTorqueTarget()
            break;

        case STATE_HALL_CALIB:
            // Gates live: the calibration scan drives the PWM open-loop from
            // the 10 kHz ISR (see hall_calibration.c). The software
            // overcurrent trip stays armed throughout.
            __HAL_TIM_MOE_ENABLE(&PWM_TIMER_HANDLE);
            break;

        case STATE_FAULT:
            // Latched protection trip: keep the gates dead until an explicit
            // operator request returns the system to IDLE.
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&PWM_TIMER_HANDLE);
            MotorControl_SetTorqueTarget(0.0f);
            break;

        default:
            // Fallback safety
            current_state = STATE_IDLE;
            break;
    }
}
void StateMachine_RequestState(MotorState_t new_state) {
    // A latched fault may only be cleared by an explicit return to IDLE
    if (current_state == STATE_FAULT && new_state != STATE_IDLE) {
        return;
    }
    current_state = new_state;
}

MotorState_t StateMachine_GetState(void) {
    return current_state;
}