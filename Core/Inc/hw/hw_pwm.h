#ifndef HW_PWM_H
#define HW_PWM_H

#include "main.h"
#include "core/foc_math.h"

// Hardware specific constants matched to your CubeMX configuration
#define PWM_ARR          1680.0f 
#define PWM_TIMER_HANDLE htim1

extern TIM_HandleTypeDef PWM_TIMER_HANDLE;

void HW_PWM_SetDuties(const Phase_t *duty);

#endif