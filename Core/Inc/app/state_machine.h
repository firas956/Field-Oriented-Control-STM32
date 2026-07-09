#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "main.h"

// Simplified operational states
typedef enum {
    STATE_IDLE = 0,       // Safe state: Gates disabled, zero torque
    STATE_CALIBRATION,    // Blocking state: Calibrating ADC offsets
    STATE_RUNNING         // Active state: Gates enabled, FOC running
} MotorState_t;

// Public Supervisor API
void StateMachine_Init(void);
void StateMachine_Update(void);
void StateMachine_RequestState(MotorState_t new_state);
MotorState_t StateMachine_GetState(void);

#endif // STATE_MACHINE_H