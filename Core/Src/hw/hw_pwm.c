#include "hw/hw_pwm.h"

void HW_PWM_SetDuties(const Phase_t *duty) {
    // Transform normalized 0.0 -> 1.0 float to Timer Ticks
    uint32_t ccr1 = (uint32_t)(duty->a * PWM_ARR);
    uint32_t ccr2 = (uint32_t)(duty->b * PWM_ARR);
    uint32_t ccr3 = (uint32_t)(duty->c * PWM_ARR);

    // Directly write to the hardware registers
    __HAL_TIM_SET_COMPARE(&PWM_TIMER_HANDLE, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&PWM_TIMER_HANDLE, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&PWM_TIMER_HANDLE, TIM_CHANNEL_3, ccr3);
}